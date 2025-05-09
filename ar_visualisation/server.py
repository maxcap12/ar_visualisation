from rclpy.node import Node
from geometry_msgs.msg import Twist
from ar_visualisation_msgs.msg import MeshesData, MarkerData
import asyncio
import websockets
import threading
import json
import socket
import time

class ServerNode(Node):
    def __init__(self, name: str, host: str, port: int):
        super().__init__(name)

        self.pub_ = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.wall_sub_ = self.create_subscription(
            MeshesData, "/ar_visualisation/mesh_data", self.wall_callback, 10
        )

        self.marker_sub_ = self.create_subscription(
            MarkerData, "/ar_visualisation/marker_data", self.marker_callback, 10
        )

        self.host = host
        self.port = port

        self.connection = None
        self.server = None
        self.loop = None
        self.running = True

        self.ws_thread = threading.Thread(target=self.run_async_server)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def show_server_ip(self):
        """
        Prints ip address of the server
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            server_ip = s.getsockname()[0]
            s.close()
            self.get_logger().info(f"server ready at host {server_ip} and port {self.port}")
        except Exception:
            self.get_logger().info(f"server ready at host localhost and port {self.port}")

    def run_async_server(self):
        """
        Runs server
        """
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def start_server():
            server = await websockets.serve(
                self.websocket_handler, self.host, self.port
            )
            self.show_server_ip()
            await asyncio.Future()
            
        try:
            self.loop.run_until_complete(start_server())

        except asyncio.CancelledError:
            self.get_logger().info("server task was canceled")
            
        finally:
            self.get_logger().info("exiting websocket thread")

    async def websocket_handler(self, websocket):
        """
        Websocket callback, publishes the data received, sends acknowledgement to client
        """
        if self.connection is not None:
            self.get_logger().warn("new connection received, ignoring")
            return
        
        self.connection = websocket
        self.get_logger().info("client connected!")

        try:
            # Instead of iterating over websocket directly
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg = Twist()
                    msg.linear.x = data["linear"]["x"] * 0.2
                    msg.linear.y = data["linear"]["y"] * 0.2
                    msg.linear.z = data["linear"]["z"] * 0.2
                    msg.angular.z = data["angular"]["z"] * 0.2

                    self.pub_.publish(msg)
                
                except json.JSONDecodeError:
                    self.get_logger().error("Received invalid JSON message")
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {e}")

        except websockets.exceptions.ConnectionClosed:
            pass
        
        finally:
            self.connection = None
            self.get_logger().info("client disconnected")

    def wall_callback(self, msg: MeshesData):
        """
        Transforms the message received into a JSON, then sends it through the websocket
        """
        data = {
            "type": "wall",
            "timestamp": time.time(),
            "data": [
                {
                    "id": mesh.id,
                    "vertices": [{"x": v.x, "y": v.y, "z": v.z} for v in mesh.vertices],
                    "triangles": list(mesh.triangles)
                }
                for mesh in msg.meshes
            ]
        }

        self.send(data)
        

    def marker_callback(self, msg: MarkerData):
        """
        Transforms the message received into a JSON, then send it through the websocket
        """
        data = {
            "type": "marker",
            "timestamp": time.time(),
            "data": {
                "id": msg.id,
                "action": msg.action,
                "type": msg.type,
                "position": {
                    "x": msg.position.x, "y": msg.position.y, "z": msg.position.z
                },
                "markerScale": {
                    "x": msg.marker_scale.x, "y": msg.marker_scale.y, "z": msg.marker_scale.z
                },
                "markerColor": {
                    "r": msg.marker_color.r, "g": msg.marker_color.g, "b": msg.marker_color.b, "a": msg.marker_color.a
                },
                "lines": [{
                    "x": line.x, "y": line.y, "z": line.z
                } for line in msg.lines ],
                "linesScale": {
                    "x": msg.lines_scale.x, "y": msg.lines_scale.y, "z": msg.lines_scale.z
                },
                "linesColor": {
                    "r": msg.lines_color.r, "g": msg.lines_color.g, "b": msg.lines_color.b, "a": msg.lines_color.a
                }
            }
        }

        self.send(data)

    def send(self, data):
        """
        Sends data through the websocket
        """
        if self.connection is None:
            self.get_logger().info("data received but no connection to client")
            return
        
        async def send_data():
                try:
                    await self.connection.send(json.dumps(data))
                    self.get_logger().info("data sent")

                except websockets.exceptions.ConnectionClosed:
                    self.get_logger().warn("client deconnected while trying to send data")
                    self.connection = None
        
        if self.loop and self.connection:
            future = asyncio.run_coroutine_threadsafe(send_data(), self.loop)

    def stop(self):
        """
        Properly stops server
        """
        self.get_logger().info("server stopping...")
        self.running = False
        
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.cancel_tasks(), self.loop)
            
            self.loop.call_soon_threadsafe(self.loop.stop)

            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join(timeout=1.)

    async def cancel_tasks(self):
        """
        Cancels running tasks
        """
        for task in asyncio.all_tasks(self.loop):
            task.cancel()

        if self.connection:
            await self.connection.close()
            self.connection = None