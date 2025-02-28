from rclpy.node import Node
from situational_graphs_msgs.msg import MeshesData
import json
import websockets
import asyncio
import threading

class ClientNode(Node):
    def __init__(self, name: str, url: str):
        super().__init__(name)

        self.sub_ = self.create_subscription(
            MeshesData, "/ar_visualisation/mesh_data", self.callback, 10
        )
        
        self.ws = None
        self.url = url
        self.connected = False
        self.connect_lock = threading.Lock()
        self.send_lock = threading.Lock()

        self.loop = None
        self.async_thread = threading.Thread(target=self.run_loop)
        self.async_thread.daemon = True
        self.async_thread.start()

        self.create_timer(5., self.check_connection)

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
        except Exception as e:
            self.get_logger().warn(f"connection failed: {e}, retrying in 5s...")
            await asyncio.sleep(5)

    def callback(self, msg: MeshesData):
        if not self.connected: return

        data = [
            {
                "id": mesh.id,
                "vertices": mesh.vertices,
                "triangles": mesh.triangles
            }
            for mesh in msg.meshes
        ]
        
        future = asyncio.run_coroutine_threadsafe(
            self.__async_send(data), self.loop
        )

        try:
            future.result(timeout=1.)
        except Exception as e:
            self.get_logger().error(f"error sending data: {e}")

    async def __async_send(self, data):
        with self.send_lock:
            try:
                await self.ws.send(json.dumps(data))
                return await self.ws.recv()
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info("connection lost...")
                self.connected = False
                return None
            except Exception as e:
                self.get_logger().error(f"error in async send: {e}")
                self.connected = False
                return None
            
    def stop(self):
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)

            if self.async_thread and self.async_thread.is_alive():
                self.async_thread.join(timeout=1.)
