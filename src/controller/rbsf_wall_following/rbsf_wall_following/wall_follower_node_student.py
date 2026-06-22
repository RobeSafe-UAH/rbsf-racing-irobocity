"""Wall-following exercises for the iRobocity course."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


LATERAL_ANGLE_RAD = math.pi / 2.0
FORWARD_ANGLE_RAD = math.pi / 3.0


class WallFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("wall_follower")

        # ------------------------------------------------------------------
        # Exercise 0: Controller initialization
        # ------------------------------------------------------------------
        # TODO: Fill in the topics, vehicle references and controller gains.
        self.drive_topic = ...
        self.scan_topic = ...

        self.reference_distance = ...
        self.speed = ...

        self.distance_kp = ...
        self.distance_ki = ...
        self.distance_integral_limit = 1.0

        self.orientation_kp = ...
        self.orientation_ki = ...
        self.orientation_integral_limit = 1.0

        return  # Remove this line after completing Exercise 0.

        self.previous_time = None
        self.distance_error_integral = 0.0
        self.orientation_error_integral = 0.0

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
        # TODO: Convert LATERAL_ANGLE_RAD to a LaserScan index, read its range,
        #       convert it to Cartesian coordinates and compute the error.
        lateral_index = round(
            (... - scan.angle_min) / scan.angle_increment
        )
        lateral_distance = scan.ranges[...]
        x_lat = ...
        y_lat = ...
        de = ...

        return  # Remove this line after completing Exercise 2.

        # ------------------------------------------------------------------
        # Exercise 3: Forward-left ray and orientation error
        # ------------------------------------------------------------------
        # TODO: Repeat the conversion for FORWARD_ANGLE_RAD and estimate the
        #       wall orientation from the two Cartesian points.
        forward_index = round(
            (... - scan.angle_min) / scan.angle_increment
        )
        forward_distance = scan.ranges[...]
        x_fwd = ...
        y_fwd = ...

        wall_orientation = math.atan2(..., ...)
        oe = ...

        return  # Remove this line after completing Exercise 3.

        # ------------------------------------------------------------------
        # Exercise 4: PI steering
        # ------------------------------------------------------------------
        # TODO: Integrate both errors using dt.
        if math.isfinite(dt) and dt > 0.0:
            self.distance_error_integral += ...
            self.orientation_error_integral += ...

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

        # TODO: Compute and combine both PI contributions.
        distance_steering = ...
        orientation_steering = ...
        steering = ...

        return  # Remove this line after completing Exercise 4.

        # ------------------------------------------------------------------
        # Exercise 5: Ackermann command
        # ------------------------------------------------------------------
        # TODO: Fill and publish the Ackermann command.
        msg = AckermannDriveStamped()
        msg.header.stamp = scan.header.stamp
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
