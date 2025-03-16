from rclpy.node import Node
from situational_graphs_msgs.msg import MeshesData
import json
import websockets
import asyncio
import threading
import uuid
import time
from collections import deque

class ClientNode(Node):
    def __init__(self, name: str, host: str, port: int, max_retries=3, retry_delay=2.0):
        super().__init__(name)

        self.sub_ = self.create_subscription(
            MeshesData, "/ar_visualisation/mesh_data", self.callback, 10
        )
        
        self.ws = None
        self.host = host
        self.port = port
        self.connected = False
        self.connect_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        
        self.pending_acks = {}
        self.message_queue = deque()
        
        self.loop = None
        self.async_thread = threading.Thread(target=self.run_loop)
        self.async_thread.daemon = True
        self.async_thread.start()

        self.create_timer(5.0, self.check_connection)
        self.create_timer(1.0, self.retry_pending_messages)

    def run_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def check_connection(self):
        if not self.connected:
            with self.connect_lock:
                if not self.connected:
                    self.get_logger().info("connection failed, attempting to connect...")
                    asyncio.run_coroutine_threadsafe(self.__async__connect(), self.loop)

    async def __async__connect(self):
        """
        Tries to connect to the server
        """
        try:
            url = f"ws://{self.host}:{self.port}"
            self.get_logger().info(f"connecting to {url}...")
            self.ws = await websockets.connect(url)
            self.connected = True
            self.get_logger().info("connected!")
            asyncio.run_coroutine_threadsafe(self.__async__listen(), self.loop)

        except Exception as e:
            self.get_logger().warn(f"connection failed: {e}, retrying in 5s...")
            await asyncio.sleep(5)

    def callback(self, msg: MeshesData):
        """
        Transforms the message received into a JSON, then sends it through the websocket
        """
        if not self.connected:
            return

        msg_id = str(uuid.uuid4())
        data = {
            "msg_id": msg_id,
            "timestamp": time.time(),
            "data": [
                {
                    "id": mesh.id,
                    "vertices": [{"x": vec.x, "y": vec.y, "z": vec.z} for vec in mesh.vertices],
                    "triangles": list(mesh.triangles)
                }
                for mesh in msg.meshes
            ]
        }
        
        self.message_queue.append((msg_id, data))
        self._process_message_queue()

    def _process_message_queue(self):
        """
        Processes the next message in the queue
        """
        if not self.message_queue or not self.connected:
            return
            
        future = asyncio.run_coroutine_threadsafe(
            self._send_next_message(), self.loop
        )

    async def _send_next_message(self):
        """
        Sends the next message in the quee
        """
        if not self.message_queue:
            return
            
        with self.send_lock:
            try:
                msg_id, data = self.message_queue.popleft()
                
                self.pending_acks[msg_id] = {
                    'data': data,
                    'retries': 0,
                    'timestamp': time.time()
                }
                
                await self.__async_send(data)

            except Exception as e:
                self.get_logger().error(f"Error sending message: {e}")

    async def __async_send(self, data):
        """
        Sends message through websocket
        """
        try:
            await self.ws.send(json.dumps(data))

        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("connection lost during send...")
            self.connected = False
            
            if 'msg_id' in data and data['msg_id'] in self.pending_acks:
                msg_data = self.pending_acks[data['msg_id']]['data']
                self.message_queue.appendleft((data['msg_id'], msg_data))
                del self.pending_acks[data['msg_id']]

        except Exception as e:
            self.get_logger().error(f"error in async send: {e}")
            self.connected = False

    async def __async__listen(self):
        """
        Checks server acknowledgement
        """
        try:
            while self.connected:
                try:
                    response = await self.ws.recv()
                    resp_data = json.loads(response)
                    
                    if 'msg_id' in resp_data and resp_data.get('status') == 'ok':
                        msg_id = resp_data['msg_id']
                        if msg_id in self.pending_acks:
                            del self.pending_acks[msg_id]
                    
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"Received invalid JSON response: {response}, error: {e}")
                
                except Exception as e:
                    self.get_logger().error(f"Error processing response: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("connection closed during listen...")
            self.connected = False

        except Exception as e:
            self.get_logger().error(f"error in async listen: {e}")
            self.connected = False

        finally:
            self.get_logger().info("Listener task ended")
    
    def retry_pending_messages(self):
        """
        Retries sending message when valid acknowledgement was not received
        """
        if not self.connected or not self.pending_acks:
            return
            
        current_time = time.time()
        to_retry = []
        
        for msg_id, info in list(self.pending_acks.items()):
            if current_time - info['timestamp'] > self.retry_delay:
                if info['retries'] < self.max_retries:
                    to_retry.append((msg_id, info['data']))
                    info['retries'] += 1
                    info['timestamp'] = current_time

                else:
                    self.get_logger().warn(f"Message {msg_id} failed after {self.max_retries} attempts, dropping")
                    del self.pending_acks[msg_id]
        
        for msg_id, data in to_retry:
            asyncio.run_coroutine_threadsafe(
                self.__async_send(data), self.loop
            )
            
    def stop(self):
        """
        Properly stops the client
        """
        if self.pending_acks:
            self.get_logger().warn(f"{len(self.pending_acks)} messages did not receive acknowledgement")
        
        if self.ws and self.connected:
            close_future = asyncio.run_coroutine_threadsafe(
                self.ws.close(), self.loop
            )
            
            try:
                close_future.result(timeout=1.0)

            except Exception as e:
                self.get_logger().warn(f"Timeout or error while closing websocket: {e}")
            
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)

            if self.async_thread and self.async_thread.is_alive():
                self.async_thread.join(timeout=1.)
