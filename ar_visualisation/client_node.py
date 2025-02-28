#!/usr/bin/env python3
import rclpy
from ar_visualisation.client import ClientNode

def main():
    rclpy.init()
    node = ClientNode("client_node", "ws://localhost:8765")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
