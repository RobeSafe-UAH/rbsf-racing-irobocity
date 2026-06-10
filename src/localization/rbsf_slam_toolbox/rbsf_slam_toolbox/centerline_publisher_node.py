import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class CenterlinePublisher(Node):
    def __init__(self):
        super().__init__('centerline_publisher')

        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/centerline')

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        topic    = self.get_parameter('topic').get_parameter_value().string_value

        if not csv_path:
            self.get_logger().error('Parameter "csv_path" is required but was not set.')
            raise RuntimeError('csv_path not set')

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Path, topic, latched_qos)

        path_msg = self._build_path(csv_path, frame_id)
        self._pub.publish(path_msg)
        self.get_logger().info(
            f'Published centerline with {len(path_msg.poses)} poses on "{topic}"'
        )

    def _build_path(self, csv_path: str, frame_id: str) -> Path:
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        if data.ndim == 1:
            data = data[np.newaxis, :]

        x = data[:, 0]
        y = data[:, 1]

        # Yaw from consecutive differences, wrap-around for closed loop
        dx = np.diff(x, append=x[0:1])
        dy = np.diff(y, append=y[0:1])
        yaw = np.arctan2(dy, dx)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for xi, yi, yi_yaw in zip(x, y, yaw):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(xi)
            pose.pose.position.y = float(yi)
            pose.pose.position.z = 0.0
            half = yi_yaw / 2.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(half)
            pose.pose.orientation.w = math.cos(half)
            path.poses.append(pose)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = CenterlinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
