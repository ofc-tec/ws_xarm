#!/usr/bin/env python3
import time

import py_trees
import rclpy
import py_trees_ros
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from xarm_bt.trees.grasping_tree import create_behavior_tree


class GraspingTreeNode(Node):
    def __init__(self):
        super().__init__("xarm_bt_grasping_tree")
        self.declare_parameter("start_delay_sec", 20.0)
        self.declare_parameter("use_introspection", False)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.use_introspection = bool(self.get_parameter("use_introspection").value)
        self.first_tick_time = time.monotonic() + self.start_delay_sec
        self.waiting_logged = False

        built_tree = create_behavior_tree(self)
        if self.use_introspection:
            self.tree = py_trees_ros.trees.BehaviourTree(built_tree.root)
            self.tree.setup(node=None, node_name="tree", timeout=15.0)
            self.get_logger().info("[xarm_bt] Grasping tree ready (introspection node: /tree)")
        else:
            self.tree = built_tree
            self.tree.setup(node=self, timeout=15.0)
            self.get_logger().info("[xarm_bt] Grasping tree ready")

    def tick_once(self):
        if time.monotonic() < self.first_tick_time:
            if not self.waiting_logged:
                self.get_logger().info(f"[xarm_bt] Waiting {self.start_delay_sec:.1f}s before first tree tick")
                self.waiting_logged = True
            return py_trees.common.Status.RUNNING

        self.tree.tick()
        return self.tree.root.status


def main(args=None):
    rclpy.init(args=args)
    node = GraspingTreeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    if node.use_introspection and getattr(node.tree, "node", None) is not None:
        executor.add_node(node.tree.node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            status = node.tick_once()
            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                node.get_logger().info(f"[xarm_bt] Grasping tree finished with {status}")
                break
            time.sleep(0.05)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
