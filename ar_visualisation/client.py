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
    def __init__(self, name: str, url: str, max_retries=3, retry_delay=2.0):
        super().__init__(name)

        self.sub_ = self.create_subscription(
            MeshesData, "/ar_visualisation/mesh_data", self.callback, 10
        )
        
        self.ws = None
        self.url = url
        self.connected = False
        self.connect_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        
        # Message queue for storing messages that need ACK
        self.pending_acks = {}
        self.message_queue = deque()
        
        # Event loop setup
        self.loop = None
        self.async_thread = threading.Thread(target=self.run_loop)
        self.async_thread.daemon = True
        self.async_thread.start()

        # Timers
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
        try:
            self.get_logger().info(f"connecting to {self.url}...")
            self.ws = await websockets.connect(self.url)
            self.connected = True
            self.get_logger().info("connected!")
            
            # Start the listener task - IMPORTANT: This needs to be properly started
            asyncio.run_coroutine_threadsafe(self.__async__listen(), self.loop)
        except Exception as e:
            self.get_logger().warn(f"connection failed: {e}, retrying in 5s...")
            await asyncio.sleep(5)

    def callback(self, msg: MeshesData):
        if not self.connected:
            return

        # Create message with ID
        msg_id = str(uuid.uuid4())
        data = {
            "msg_id": msg_id,
            "timestamp": time.time(),
            "data": [
                {
                    "id": mesh.id,
                    "vertices": [{"x": vec.x, "y": vec.y, "z": vec.z} for vec in mesh.vertices],
                    "triangles": mesh.triangles
                }
                for mesh in msg.meshes
            ]
        }
        
        self.get_logger().info(f"Queuing message with ID: {msg_id}")
        
        # Add to pending queue and send
        self.message_queue.append((msg_id, data))
        self._process_message_queue()

    def _process_message_queue(self):
        """Process the next message in the queue if possible"""
        if not self.message_queue or not self.connected:
            return
            
        # Process the next message
        future = asyncio.run_coroutine_threadsafe(
            self._send_next_message(), self.loop
        )

    async def _send_next_message(self):
        """Send the next message from the queue"""
        if not self.message_queue:
            return
            
        with self.send_lock:
            try:
                msg_id, data = self.message_queue.popleft()
                
                # Track this message for ACK
                self.pending_acks[msg_id] = {
                    'data': data,
                    'retries': 0,
                    'timestamp': time.time()
                }
                
                self.get_logger().info(f"Sending message {msg_id} and waiting for ACK")
                await self.__async_send(data)
            except Exception as e:
                self.get_logger().error(f"Error sending message: {e}")

    async def __async_send(self, data):
        """Send data to the websocket server"""
        try:
            msg_id = data.get('msg_id', 'unknown')
            self.get_logger().info(f"Sending message {msg_id} with {len(data['data'])} meshes")
            await self.ws.send(json.dumps(data))
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("connection lost during send...")
            self.connected = False
            # Put the message back in the queue if it has a message ID
            if 'msg_id' in data and data['msg_id'] in self.pending_acks:
                msg_data = self.pending_acks[data['msg_id']]['data']
                self.message_queue.appendleft((data['msg_id'], msg_data))
                del self.pending_acks[data['msg_id']]
        except Exception as e:
            self.get_logger().error(f"error in async send: {e}")
            self.connected = False

    async def __async__listen(self):
        """Listen for responses from the server"""
        self.get_logger().info("Starting listener for server responses")
        try:
            while self.connected:
                try:
                    self.get_logger().info("Waiting for server response...")
                    response = await self.ws.recv()
                    self.get_logger().info(f"Received response: {response}")
                    
                    resp_data = json.loads(response)
                    
                    # Check for acknowledgment
                    if 'msg_id' in resp_data and resp_data.get('status') == 'ok':
                        msg_id = resp_data['msg_id']
                        if msg_id in self.pending_acks:
                            self.get_logger().info(f"✓ Received ACK for message {msg_id}")
                            del self.pending_acks[msg_id]
                        else:
                            self.get_logger().warn(f"Received ACK for unknown message {msg_id}")
                    
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
        """Retry sending messages that haven't been acknowledged"""
        if not self.connected or not self.pending_acks:
            return
            
        current_time = time.time()
        to_retry = []
        
        if self.pending_acks:
            self.get_logger().info(f"Currently waiting for {len(self.pending_acks)} acknowledgements")
        
        # Identify messages that need to be retried
        for msg_id, info in list(self.pending_acks.items()):
            # If it's been more than retry_delay seconds since sending
            if current_time - info['timestamp'] > self.retry_delay:
                if info['retries'] < self.max_retries:
                    self.get_logger().info(f"Retrying message {msg_id} (attempt {info['retries'] + 1}/{self.max_retries})")
                    to_retry.append((msg_id, info['data']))
                    
                    # Update retry info
                    info['retries'] += 1
                    info['timestamp'] = current_time
                else:
                    self.get_logger().warn(f"Message {msg_id} failed after {self.max_retries} attempts, dropping")
                    del self.pending_acks[msg_id]
        
        # Queue the retries
        for msg_id, data in to_retry:
            asyncio.run_coroutine_threadsafe(
                self.__async_send(data), self.loop
            )
            
    def stop(self):
        # Log any pending messages that didn't get acknowledgements
        if self.pending_acks:
            self.get_logger().warn(f"{len(self.pending_acks)} messages did not receive acknowledgement")
        
        # Properly close the websocket connection
        if self.ws and self.connected:
            close_future = asyncio.run_coroutine_threadsafe(
                self.ws.close(), self.loop
            )
            try:
                close_future.result(timeout=1.0)
            except Exception as e:
                self.get_logger().warn(f"Timeout or error while closing websocket: {e}")
            
        # Stop the event loop
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)

            if self.async_thread and self.async_thread.is_alive():
                self.async_thread.join(timeout=1.)