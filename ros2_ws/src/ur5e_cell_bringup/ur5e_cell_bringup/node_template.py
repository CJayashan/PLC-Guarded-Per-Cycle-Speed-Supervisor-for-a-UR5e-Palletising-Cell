#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MyNode(Node):
    def __init__(self) -> None:
        # 1) Name your node (shows up in `ros2 node list`)
        super().__init__("my_node_name")

        # 2) TODO: create publishers, subscribers, timers here
        # self.pub = ...
        # self.sub = ...

    # 3) TODO: add callbacks here
    # def some_callback(self, msg):
    #     pass


def main(args=None) -> None:
    # 4) Standard ROS 2 node startup
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
