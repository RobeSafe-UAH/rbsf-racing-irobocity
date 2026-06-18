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
        # 1. Parameters
        # ------------------------------------------------------------------
        # ROS topics.
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.drive_topic = self.declare_parameter("drive_topic", "/drive").value

        # Vehicle references.
        self.desired_distance = self.declare_parameter(
            "desired_distance", 0.7
        ).value
        self.speed = self.declare_parameter("speed", 0.6).value

        # Distance controller gains.
        self.distance_kp = self.declare_parameter("distance_kp", 1.0).value
        self.distance_ki = self.declare_parameter("distance_ki", 0.0).value

        # Orientation controller gains.
        self.orientation_kp = self.declare_parameter("orientation_kp", 1.0).value
        self.orientation_ki = self.declare_parameter("orientation_ki", 0.0).value

        # Previous LiDAR timestamp, used to estimate the sampling period.
        self.previous_time = None

        # ------------------------------------------------------------------
        # 2. ROS interfaces
        # ------------------------------------------------------------------
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
        # 1. Sampling period.
        # ------------------------------------------------------------------
        stamp = scan.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        if self.previous_time is None:
            dt = scan.scan_time
        else:
            dt = current_time - self.previous_time

        self.previous_time = current_time

        # ------------------------------------------------------------------
        # 2. Lateral ray.
        # ------------------------------------------------------------------
        # This ray points directly towards the left wall.
        lateral_index = round(
            (LATERAL_ANGLE_RAD - scan.angle_min) / scan.angle_increment
        )
        lateral_distance = scan.ranges[lateral_index]
        lateral_x = lateral_distance * math.cos(LATERAL_ANGLE_RAD)
        lateral_y = lateral_distance * math.sin(LATERAL_ANGLE_RAD)

        # ------------------------------------------------------------------
        # 3. Forward-left ray.
        # ------------------------------------------------------------------
        # Together with the lateral ray, this point defines an approximation
        # of the local wall direction.
        forward_index = round(
            (FORWARD_ANGLE_RAD - scan.angle_min) / scan.angle_increment
        )
        forward_distance = scan.ranges[forward_index]
        forward_x = forward_distance * math.cos(FORWARD_ANGLE_RAD)
        forward_y = forward_distance * math.sin(FORWARD_ANGLE_RAD)

        # ------------------------------------------------------------------
        # 4. Wall distance and wall orientation.
        # ------------------------------------------------------------------
        # The y coordinate of the lateral point is the left-wall distance.
        wall_distance = abs(lateral_y)

        # The orientation is obtained from the segment joining the two points
        # measured by the LiDAR rays.
        wall_orientation = math.atan2(
            forward_y - lateral_y,
            forward_x - lateral_x,
        )

        # ------------------------------------------------------------------
        # 5. Control errors.
        # ------------------------------------------------------------------
        # de is positive when the vehicle is farther than the reference
        # distance from the left wall.
        de = wall_distance - self.desired_distance

        # oe is zero when the longitudinal axis of the vehicle is parallel to
        # the estimated wall direction.
        oe = wall_orientation

        # ------------------------------------------------------------------
        # 6. Steering command.
        # ------------------------------------------------------------------
        # Each error contributes to the final steering command.
        distance_steering = de * self.distance_kp + de * self.distance_ki * dt
        orientation_steering = (
            oe * self.orientation_kp
            + oe * self.orientation_ki * dt
        )

        steering = distance_steering + orientation_steering

        # ------------------------------------------------------------------
        # 7. Ackermann command.
        # ------------------------------------------------------------------
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
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
