from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import threading
import json
import socket
import time
import importlib
import array
import struct
import numpy as np
import cv2

class ServerNode(Node):
    def __init__(self, name: str, host: str, port: int):
        super().__init__(name)

        self.pub_ = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.subscriptions_ = {}

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
            async for message in websocket:
                try:
                    data = json.loads(message)

                    match data["type"]:
                        case "geometry_msgs/Twist":
                            msg = Twist()
                            content = data["content"]
                            msg.linear.x = content["linear"]["x"] * 0.2
                            msg.linear.y = content["linear"]["y"] * 0.2
                            msg.linear.z = content["linear"]["z"] * 0.2
                            msg.angular.z = content["angular"]["z"] * 0.2
                            self.pub_.publish(msg)
                        
                        case "subscribe":
                            self.add_subscription(data["content"])

                        case "unsubscribe":
                            self.remove_subscription(data["content"])

                        case _:
                            # if more types are necessary, the implementation should be changed
                            self.get_logger().info(f"unsuported message type: {data['type']}")
                
                except json.JSONDecodeError:
                    self.get_logger().error("Received invalid JSON message")
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {e}")

        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().info(e)
        
        finally:
            self.connection = None
            self.get_logger().info("client disconnected")
            self.clear_subscriptions()

    def add_subscription(self, topic_name):
        """
        Subscribes to a topic if it exists
        """
        def get_msg_class(type_str):
            pkg, _, rest = type_str.partition('/')
            subfolder, _, msg_type = rest.partition('/')
            module_name = f"{pkg}.{subfolder}"
            return getattr(importlib.import_module(module_name), msg_type)

        if topic_name in self.subscriptions_.keys(): 
            self.get_logger().info(f"a subscription to topic {topic_name} already exists")
            return

        topics_info = self.get_topic_names_and_types()

        for topic_info in topics_info:

            if (topic_info[0] == topic_name):

                if (len(topic_info[1]) != 1):
                    self.get_logger().info(f"topic {topic_name} has multiple message types: {';'.join(topic_info[1])}")
                    return
                
                msg_class = get_msg_class(topic_info[1][0])
                
                self.subscriptions_[topic_name] = self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda msg: self.msg_callback(msg, topic_name, "Image" in msg_class),
                    10
                )
                self.get_logger().info(f"subscribed to topic {topic_name}")
                return
            
        self.get_logger().info(f"topic {topic_name} not found, subscribe process aborted")

    def remove_subscription(self, topic_name):
        """
        Unsubscribes from a topic
        """
        if topic_name not in self.subscriptions_.keys(): return

        self.destroy_subscription(self.subscriptions_.pop(topic_name))
        self.get_logger().info(f"unsubscribed from topic {topic_name}")

    def clear_subscriptions(self):
        """
        Stops all subscriptions
        """
        topics = list(self.subscriptions_.keys())

        for topic_name in topics:
            self.remove_subscription(topic_name)

    def msg_callback(self, msg, source, is_img):
        """
        Callback used for every subscription
        Transforms the message into a json and sends it over the websocket
        """
        def msg_to_dict(message):
            if hasattr(message, '__slots__'):
                return {
                    field[1:] if field[0] == '_' else field: 
                    msg_to_dict(getattr(message, field)) for field in message.__slots__
                }
            
            elif isinstance(message, array.array):
                return list(message)
            
            elif isinstance(message, (list, tuple)):
                return [msg_to_dict(v) for v in message]
            
            else:
                return message
            
        def msg_to_binary(message, source: str):
            dtype = np.uint8
            if "32" in msg.encoding:
                dtype = np.float32
            elif "16" in msg.encoding:
                dtype = np.uint16
                
            img = np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, -1
            )
            
            if msg.encoding == "bgr8":
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]  # Quality: 80%
            _, compressed_data = cv2.imencode(".jpg", img, encode_param)
            data = compressed_data.tobytes()
            
            topic_bytes = source.encode("utf-8")
            encoding_bytes = message.encoding.encode("utf-8")

            header = struct.pack(
                ">IIIIII",
                len(topic_bytes), 
                len(encoding_bytes), 
                message.height, 
                message.width,
                message.step,
                len(data),
            )
            return (
                header +
                topic_bytes +
                encoding_bytes +
                data
            )

        if is_img:
            self.send(
                msg_to_binary(msg, source),
                False
            )
        else:
            self.send({
                "type": source,
                "timestamp": int(time.time() * 1000),
                "content": msg_to_dict(msg)
                },
                True
            )

    def send(self, data, is_json):
        """
        Sends data as a text frame through the websocket
        """
        if self.connection is None:
            self.get_logger().info("data received but no connection to client")
            return
        
        async def send_data():
            try:
                if is_json:
                    await self.connection.send(json.dumps(data))
                
                else:
                    await self.connection.send(data)
                    
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
        self.clear_subscriptions()
        
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