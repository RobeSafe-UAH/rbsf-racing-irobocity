"""Resolved wall-following controller for the iRobocity course.

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
# Geometry constants
# --------------------------------------------------------------------------
# The ray angles are fixed in the source code to keep the geometric
# construction explicit.
LATERAL_ANGLE_RAD = math.pi / 2.0
FORWARD_ANGLE_RAD = math.pi / 3.0


class WallFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower")

        # ------------------------------------------------------------------
        # Controller configuration
        # ------------------------------------------------------------------

        self.drive_topic = "/drive"
        self.scan_topic = "/scan"

        # Vehicle references.
        self.reference_distance = 0.7
        self.speed = 0.6

        # Distance-controller gains and integral limit.
        self.distance_kp = 1.0
        self.distance_ki = 0.0
        self.distance_integral_limit = 1.0

        # Orientation-controller gains and integral limit.
        self.orientation_kp = 1.0
        self.orientation_ki = 0.0
        self.orientation_integral_limit = 1.0

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
        # Sampling period.
        # ------------------------------------------------------------------

        stamp = scan.header.stamp
        current_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if self.previous_time is None:
            dt = scan.scan_time
        else:
            dt = current_time - self.previous_time

        self.previous_time = current_time

        # ------------------------------------------------------------------
        # Lateral ray and distance error.
        # ------------------------------------------------------------------

        # This ray points directly towards the left wall.
        lateral_index = round(
            (LATERAL_ANGLE_RAD - scan.angle_min) / scan.angle_increment
        )
        lateral_distance = scan.ranges[lateral_index]
        x_lat = lateral_distance * math.cos(LATERAL_ANGLE_RAD)
        y_lat = lateral_distance * math.sin(LATERAL_ANGLE_RAD)

        de = abs(y_lat) - self.reference_distance

        # ------------------------------------------------------------------
        # Forward-left ray and orientation error.
        # ------------------------------------------------------------------

        forward_index = round(
            (FORWARD_ANGLE_RAD - scan.angle_min) / scan.angle_increment
        )
        forward_distance = scan.ranges[forward_index]
        x_fwd = forward_distance * math.cos(FORWARD_ANGLE_RAD)
        y_fwd = forward_distance * math.sin(FORWARD_ANGLE_RAD)

        wall_orientation = math.atan2(
            y_fwd - y_lat,
            x_fwd - x_lat,
        )

        oe = wall_orientation

        # ------------------------------------------------------------------
        # PI steering.
        # ------------------------------------------------------------------

        # A PI controller uses the accumulated error, not only the error from
        # the current sampling period. Invalid or repeated timestamps are not
        # integrated.
        if math.isfinite(dt) and dt > 0.0:
            self.distance_error_integral += de * dt
            self.orientation_error_integral += oe * dt

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

        distance_steering = (
            de * self.distance_kp
            + self.distance_error_integral * self.distance_ki
        )
        orientation_steering = (
            oe * self.orientation_kp
            + self.orientation_error_integral * self.orientation_ki
        )

        steering = distance_steering + orientation_steering

        # ------------------------------------------------------------------
        # Ackermann command.
        # ------------------------------------------------------------------

        msg = AckermannDriveStamped()
        msg.header.stamp = scan.header.stamp
        msg.drive.speed = self.speed
        msg.drive.steering_angle = steering
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
