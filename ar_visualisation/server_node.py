#!/usr/bin/env python3
import rclpy
from ar_visualisation.server import ServerNode

def main():
    rclpy.init()
    node = ServerNode('server_node', "localhost", 8765)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    