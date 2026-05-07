#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PCRepubSanitized(Node):
    def __init__(self):
        super().__init__('pc_repub_sanitized')

        # ros_gz_bridge is RELIABLE in your current output
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.cb,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/camera/depth/points_repub',
            qos
        )

        self.count = 0

        # Temporal downsample: publish 1 out of N clouds
        self.keep_every = 2

        # Spatial downsample: keep 1 pixel every N rows/cols
        self.pixel_step = 4

        self.get_logger().info(
            f'pc_repub_sanitized running: keep_every={self.keep_every}, '
            f'pixel_step={self.pixel_step}'
        )

    def cb(self, msg):
        self.count += 1

        if self.count % self.keep_every != 0:
            return

        try:
            xyz = point_cloud2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=False
            )

            # If organized cloud, downsample in image space
            if msg.height > 1 and msg.width > 1:
                expected_points = msg.height * msg.width

                if xyz.shape[0] == expected_points:
                    xyz = xyz.reshape(msg.height, msg.width, 3)
                    xyz = xyz[::self.pixel_step, ::self.pixel_step, :]
                    xyz = xyz.reshape(-1, 3)
                else:
                    self.get_logger().warn(
                        f'Organized cloud metadata mismatch before sanitizing: '
                        f'height={msg.height}, width={msg.width}, '
                        f'xyz_points={xyz.shape[0]}. Publishing flattened cloud.'
                    )
                    xyz = xyz.reshape(-1, 3)
                    xyz = xyz[::self.pixel_step, :]

            else:
                # Already unorganized
                xyz = xyz.reshape(-1, 3)
                xyz = xyz[::self.pixel_step, :]

            # Create clean unorganized XYZ cloud.
            # IMPORTANT: do not manually set height/width afterward.
            out = point_cloud2.create_cloud_xyz32(
                msg.header,
                xyz
            )

            out.is_dense = False

            self.pub.publish(out)

            self.get_logger().info(
                f'published sanitized cloud: '
                f'in=({msg.height}x{msg.width}) '
                f'out=({out.height}x{out.width}) '
                f'points={out.width}'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to sanitize cloud: {e}')


def main():
    rclpy.init()
    node = PCRepubSanitized()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()