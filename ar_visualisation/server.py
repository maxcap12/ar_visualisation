from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import websockets
import threading
import json

class ServerNode(Node):
    def __init__(self, name: str, port: int, host: str = "0.0.0.0"):
        super().__init__(name)

        self.pub_ = self.create_publisher(
            String, "/test", 10
        )

        self.host = host
        self.port = port
        self.server = None
        self.loop = None
        self.running = True

        self.ws_thread = threading.Thread(target=self.run_async_server)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def run_async_server(self):
        """
        Runs server
        """
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.get_logger().info(f"starting server at url ws://{self.host}:{self.port}")

        async def start_server():
            server = await websockets.serve(
                self.websocket_handler, self.host, self.port
            )
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
        self.get_logger().info("client connected!")

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_id = data.get("msg_id", "unknown")
                    
                    msg = String()
                    msg.data = message
                    self.pub_.publish(msg)
                    
                    # Send acknowledgment with the message ID
                    ack_response = json.dumps({"status": "ok", "msg_id": msg_id})
                    await websocket.send(ack_response)
                
                except json.JSONDecodeError:
                    self.get_logger().error("Received invalid JSON message")
                    await websocket.send(json.dumps({"status": "error", "reason": "invalid_json"}))
                
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {e}")
                    await websocket.send(json.dumps({"status": "error", "reason": str(e)}))

        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("client disconnected")

    def stop(self):
        """
        Properly stops server
        """
        self.get_logger().info("server stopping...")
        self.running = False
        
        if self.loop:
            for task in asyncio.all_tasks(self.loop):
                task.cancel()
            
            self.loop.call_soon_threadsafe(self.loop.stop)

            if self.ws_thread and self.ws_thread.is_alive():
                self.ws_thread.join(timeout=1.)
