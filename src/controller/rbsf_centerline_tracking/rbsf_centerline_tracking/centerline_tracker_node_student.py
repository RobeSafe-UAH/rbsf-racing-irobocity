"""Centerline-tracking exercises for the iRobocity course."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class CenterlineTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("centerline_tracker_student")

        self.path_topic = "/centerline"
        self.odom_topic = "/odom"
        self.drive_topic = "/drive"

        self.speed = 1.0
        self.lookahead_distance = 1.0

        self.kp = 1.0
        self.ki = 0.0
        self.orientation_integral_limit = 1.0

        self.previous_time = None
        self.orientation_error_integral = 0.0
        self.path = None

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

    def path_callback(self, path: Path) -> None:
        self.path = path

    def odom_callback(self, odom: Odometry) -> None:
        if self.path is None or not self.path.poses:
            return

        stamp = odom.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time

        self.previous_time = current_time

        pose = odom.pose.pose
        vehicle_x = pose.position.x
        vehicle_y = pose.position.y

        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)

        closest_index = 0
        closest_distance = math.inf

        for index, path_pose in enumerate(self.path.poses):
            point = path_pose.pose.position
            distance = math.hypot(point.x - vehicle_x, point.y - vehicle_y)

            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        # ------------------------------------------------------------------
        # Exercise 1: Lookahead target
        # ------------------------------------------------------------------
        # TODO: Starting at the closest point, move forward through the cyclic
        #       path until reaching the configured lookahead distance.
        path_length = len(self.path.poses)
        target_index = ...

        for steps_ahead in range(path_length):
            index = ...
            point = self.path.poses[index].pose.position
            distance = ...

            if ...:
                target_index = ...
                break

        target = ...

        return  # Remove this line after completing Exercise 1.

        # ------------------------------------------------------------------
        # Exercise 2: Orientation error
        # ------------------------------------------------------------------
        dx = target.x - vehicle_x
        dy = target.y - vehicle_y

        # TODO: Compute the desired yaw and its difference from vehicle_yaw.
        desired_yaw = ...
        oe = ...

        # Normalize the error to [-pi, pi].
        oe = math.atan2(math.sin(oe), math.cos(oe))

        return  # Remove this line after completing Exercise 2.

        # ------------------------------------------------------------------
        # Exercise 3: PI control and Ackermann command
        # ------------------------------------------------------------------
        # TODO: Integrate the orientation error using dt.
        if math.isfinite(dt) and dt > 0.0:
            self.orientation_error_integral += ...

            self.orientation_error_integral = max(
                -self.orientation_integral_limit,
                min(
                    self.orientation_integral_limit,
                    self.orientation_error_integral,
                ),
            )

        # TODO: Compute steering, fill the command and publish it.
        steering = ...

        msg = AckermannDriveStamped()
        msg.header.stamp = odom.header.stamp
        msg.drive.speed = ...
        msg.drive.steering_angle = ...
        self.publisher.publish(...)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CenterlineTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
