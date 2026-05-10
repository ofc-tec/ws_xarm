#!/usr/bin/env python3
import time

import py_trees
import rclpy
from rclpy.node import Node

from xarm_bt.trees.grasping_tree import create_behavior_tree


class GraspingTreeNode(Node):
    def __init__(self):
        super().__init__("xarm_bt_grasping_tree")
        self.tree = create_behavior_tree(self)
        self.tree.setup(node=self, timeout=15.0)
        self.get_logger().info("[xarm_bt] Grasping tree ready")

    def tick_once(self):
        self.tree.tick()
        return self.tree.root.status


def main(args=None):
    rclpy.init(args=args)
    node = GraspingTreeNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            status = node.tick_once()
            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                node.get_logger().info(f"[xarm_bt] Grasping tree finished with {status}")
                break
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
