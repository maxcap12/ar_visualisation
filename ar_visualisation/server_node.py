#!/usr/bin/env python3
import rclpy
from ar_visualisation.server import ServerNode
import argparse

def main(args):
    rclpy.init()
    node = ServerNode("server_node", args.host, args.port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("host", type=str)
    parser.add_argument("port", type=int)
    args, _ = parser.parse_known_args()
    
    main(args)
    