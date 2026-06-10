import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("side", "right")
        self.declare_parameter("desired_distance", 0.7)
        self.declare_parameter("speed", 0.6)
        self.declare_parameter("kp", 1.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.08)
        self.declare_parameter("max_steering_angle", 0.35)
        self.declare_parameter(
            "side_angle_deg",
            90.0,
            ParameterDescriptor(description="Angle used to measure the wall distance."),
        )
        self.declare_parameter(
            "sector_width_deg",
            10.0,
            ParameterDescriptor(description="Laser sector width averaged around side_angle_deg."),
        )

        self.scan_topic = self.get_parameter("scan_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.side = self.get_parameter("side").value.lower()
        self.desired_distance = float(self.get_parameter("desired_distance").value)
        self.speed = float(self.get_parameter("speed").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.max_steering_angle = float(self.get_parameter("max_steering_angle").value)
        self.side_angle_deg = float(self.get_parameter("side_angle_deg").value)
        self.sector_width_deg = float(self.get_parameter("sector_width_deg").value)

        if self.side not in ("left", "right"):
            raise ValueError("Parameter 'side' must be 'left' or 'right'")

        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time: Optional[float] = None

        self.publisher = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        self.get_logger().info(
            f"Wall follower ready: scan={self.scan_topic}, drive={self.drive_topic}, "
            f"side={self.side}, distance={self.desired_distance:.2f} m"
        )

    def scan_callback(self, scan: LaserScan) -> None:
        distance = self.get_wall_distance(scan)
        if distance is None:
            self.publish_drive(0.0, 0.0)
            self.get_logger().warn("No valid wall distance in LaserScan; stopping.", throttle_duration_sec=1.0)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.previous_time is None else max(now - self.previous_time, 1e-3)
        self.previous_time = now

        error = self.desired_distance - distance
        self.integral += error * dt
        derivative = 0.0 if dt == 0.0 else (error - self.previous_error) / dt
        self.previous_error = error

        steering = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.side == "left":
            steering = -steering
        steering = self.clamp(steering, -self.max_steering_angle, self.max_steering_angle)

        self.publish_drive(self.speed, steering)

    def get_wall_distance(self, scan: LaserScan) -> Optional[float]:
        angle_deg = -self.side_angle_deg if self.side == "right" else self.side_angle_deg
        center = math.radians(angle_deg)
        width = math.radians(self.sector_width_deg)

        start = self.angle_to_index(scan, center - width / 2.0)
        end = self.angle_to_index(scan, center + width / 2.0)
        if start > end:
            start, end = end, start

        distances = [
            value
            for value in scan.ranges[start : end + 1]
            if math.isfinite(value) and scan.range_min <= value <= scan.range_max
        ]
        if not distances:
            return None

        return sum(distances) / len(distances)

    def angle_to_index(self, scan: LaserScan, angle: float) -> int:
        index = round((angle - scan.angle_min) / scan.angle_increment)
        return int(self.clamp(index, 0, len(scan.ranges) - 1))

    def publish_drive(self, speed: float, steering: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        self.publisher.publish(msg)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
