"""Centerline tracking exercise for the iRobocity course.

The node receives an ordered centerline and vehicle odometry, selects a
lookahead point and publishes a constant-speed Ackermann command.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class CenterlineTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("centerline_tracker")

        # ------------------------------------------------------------------
        # Controller configuration
        # ------------------------------------------------------------------
        # Both odometry topics are subscribed so the same source works on the
        # physical car and in MVSim without an external parameter file.
        self.path_topic = "/centerline"
        self.odom_topics = ("/odom", "/base_pose_ground_truth")
        self.drive_topic = "/drive"

        # Vehicle references.
        self.speed = 1.0
        self.lookahead_distance = 1.0

        # Orientation controller gains.
        self.kp = 1.0
        self.ki = 0.0
        self.orientation_integral_limit = 1.0

        # Previous odometry timestamp, used to estimate the sampling period.
        self.previous_time = None

        # Accumulated orientation error used by the integral term.
        self.orientation_error_integral = 0.0

        # Centerline path received by the node.
        self.path = None

        # ROS interfaces.
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.odom_subscriptions = [
            self.create_subscription(
                Odometry,
                topic,
                self.odom_callback,
                10,
            )
            for topic in self.odom_topics
        ]

    def path_callback(self, path: Path) -> None:
        # ------------------------------------------------------------------
        # Objective:
        #   Store the centerline path. The control action is computed when an
        #   odometry message is received.
        # ------------------------------------------------------------------
        self.path = path

    def odom_callback(self, odom: Odometry) -> None:
        # ------------------------------------------------------------------
        # Objective:
        #   Compute one Ackermann control command for each incoming odometry
        #   message.
        #
        # Method:
        #   1. Read the current vehicle pose.
        #   2. Select a lookahead target on the centerline.
        #   3. Compute the orientation error to the target point.
        #   4. Compute the steering command and publish it.
        #
        # The longitudinal speed is constant. The controller only modifies the
        # steering angle.
        # ------------------------------------------------------------------

        # The controller can only run after receiving the centerline path.
        if self.path is None or not self.path.poses:
            return

        # Sampling period.
        stamp = odom.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time

        self.previous_time = current_time

        # Vehicle pose.
        pose = odom.pose.pose
        vehicle_x = pose.position.x
        vehicle_y = pose.position.y

        # Yaw is obtained from the quaternion orientation.
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Closest centerline point.
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
        # The target vector is expressed in the global frame.
        dx = target.x - vehicle_x
        dy = target.y - vehicle_y

        # The desired yaw is the orientation of the vector that joins the
        # vehicle position with the lookahead target.
        # TODO: Compute the desired yaw towards the lookahead target.
        desired_yaw = ...

        # The orientation error is the difference between the desired yaw and
        # the current vehicle yaw.
        # TODO: Compute the orientation error.
        oe = ...

        # Normalize the error to [-pi, pi].
        oe = math.atan2(math.sin(oe), math.cos(oe))

        return  # Remove this line after completing Exercise 2.

        # ------------------------------------------------------------------
        # Exercise 3: PI control and Ackermann command
        # ------------------------------------------------------------------
        # A PI controller uses the accumulated orientation error, not only
        # the error from the current sampling period. Invalid or repeated
        # timestamps are not integrated.
        # TODO: Integrate the orientation error using dt.
        if math.isfinite(dt) and dt > 0.0:
            self.orientation_error_integral += ...

            # Clamp the accumulator to avoid integral windup.
            self.orientation_error_integral = max(
                -self.orientation_integral_limit,
                min(
                    self.orientation_integral_limit,
                    self.orientation_error_integral,
                ),
            )

        # TODO: Compute the PI steering command.
        steering = ...

        # TODO: Fill and publish the Ackermann command.
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
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
