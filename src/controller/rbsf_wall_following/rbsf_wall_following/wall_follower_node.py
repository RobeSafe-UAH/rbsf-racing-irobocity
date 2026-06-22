"""wall_follower_node.py — Wall-following exercise for the iRobocity course.

Subscribed topics (inputs):
    /scan or /laser1    sensor_msgs/LaserScan
        LiDAR scan from the physical car or MVSim.

Published topics (outputs):
    /drive              ackermann_msgs/AckermannDriveStamped
        Constant-speed command with PI-controlled steering.

Controller outline:
    1. Estimate the sampling period from LiDAR timestamps.
    2. Read one lateral and one forward-left LiDAR ray.
    3. Estimate distance and orientation relative to the left wall.
    4. Apply distance and orientation PI controllers.
    5. Publish the resulting Ackermann command.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# --------------------------------------------------------------------------
# Exercise constants
# --------------------------------------------------------------------------
# The ray angles are fixed in the source code to make the geometric
# construction explicit in the exercise.
LATERAL_ANGLE_RAD = math.pi / 2.0
FORWARD_ANGLE_RAD = math.pi / 3.0


class WallFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower")

        # ------------------------------------------------------------------
        # Exercise 0: Controller initialization
        # ------------------------------------------------------------------

        # TODO: Fill in the topics, vehicle references and controller gains
        self.drive_topic = ...
        self.scan_topic = ...

        # Vehicle references.
        self.reference_distance = ...
        self.speed = ...

        # Distance-controller gains and integral limit.
        self.distance_kp = ...
        self.distance_ki = ...
        self.distance_integral_limit = 1.0

        # Orientation-controller gains and integral limit.
        self.orientation_kp = ...
        self.orientation_ki = ...
        self.orientation_integral_limit = 1.0

        return  # Remove this line after completing Exercise 0.

        # Previous LiDAR timestamp, used to estimate the sampling period.
        self.previous_time = None

        # Accumulated errors used by the two integral terms.
        self.distance_error_integral = 0.0
        self.orientation_error_integral = 0.0

        # ROS interfaces.
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

    def scan_callback(self, scan: LaserScan) -> None:
        # ------------------------------------------------------------------
        # Objective:
        #   Compute Ackermann control command for the incoming LiDAR scan.
        #
        # Method:
        #   1. Read two LiDAR rays on the left side of the vehicle.
        #   2. Estimate wall distance and wall orientation from those rays.
        #   3. Compute the distance and orientation errors.
        #   4. Compute the steering command and publish it.
        #
        # The longitudinal speed is constant. The controller only modifies the
        # steering angle.
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # Exercise 1: Sampling period
        # ------------------------------------------------------------------

        # TODO: Compute the sampling period from the previous scan timestamp.
        stamp = scan.header.stamp
        current_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if self.previous_time is None:
            dt = ...
        else:
            dt = ...

        self.previous_time = ...

        return  # Remove this line after completing Exercise 1.

        # ------------------------------------------------------------------
        # Exercise 2: Lateral ray and distance error
        # ------------------------------------------------------------------

        # This ray points directly towards the left wall.
        # TODO: Convert LATERAL_ANGLE_RAD to a LaserScan index, read its range,
        #       and convert the polar measurement to Cartesian coordinates.
        lateral_index = round(
            (... - scan.angle_min) / scan.angle_increment
        )
        lateral_distance = scan.ranges[...]
        x_lat = ...
        y_lat = ...

        # TODO: Compute the distance error.
        de = ...

        return  # Remove this line after completing Exercise 2.

        # ------------------------------------------------------------------
        # Exercise 3: Forward-left ray and orientation error
        # ------------------------------------------------------------------

        # TODO: Repeat the index, range and Cartesian conversion for the
        #       forward-left ray at FORWARD_ANGLE_RAD.
        forward_index = round(
            (... - scan.angle_min) / scan.angle_increment
        )
        forward_distance = scan.ranges[...]
        x_fwd = ...
        y_fwd = ...

        # TODO: Compute its orientation with atan2(delta_y, delta_x).
        wall_orientation = math.atan2(
            ...,
            ...,
        )

        # TODO: Compute the orientation error.
        oe = ...

        return  # Remove this line after completing Exercise 3.

        # ------------------------------------------------------------------
        # Exercise 4: PI steering
        # ------------------------------------------------------------------

        # A PI controller uses the accumulated error, not only the error from
        # the current sampling period. Invalid or repeated timestamps are not
        # integrated.
        # TODO: Integrate both errors using the sampling period dt.
        if math.isfinite(dt) and dt > 0.0:
            self.distance_error_integral += ...
            self.orientation_error_integral += ...

            # Clamp both accumulators to avoid integral windup.
            self.distance_error_integral = max(
                -self.distance_integral_limit,
                min(
                    self.distance_integral_limit,
                    self.distance_error_integral,
                ),
            )
            self.orientation_error_integral = max(
                -self.orientation_integral_limit,
                min(
                    self.orientation_integral_limit,
                    self.orientation_error_integral,
                ),
            )

        # TODO: Compute the proportional and integral contribution of each
        #       error, then combine both steering contributions.
        distance_steering = ...
        orientation_steering = ...

        steering = ...

        return  # Remove this line after completing Exercise 4.

        # ------------------------------------------------------------------
        # Exercise 5: Ackermann command
        # ------------------------------------------------------------------

        # TODO: Fill the speed and steering fields and publish the command.
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = ...
        msg.drive.steering_angle = ...
        self.publisher.publish(...)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
