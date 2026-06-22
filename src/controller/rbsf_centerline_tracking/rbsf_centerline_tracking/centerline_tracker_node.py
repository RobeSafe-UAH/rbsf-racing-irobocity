import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class CenterlineTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("centerline_tracker")

        # ------------------------------------------------------------------
        # 1. Parameters
        # ------------------------------------------------------------------
        # ROS topics.
        self.path_topic = self.declare_parameter("path_topic", "/centerline").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.drive_topic = self.declare_parameter("drive_topic", "/drive").value

        # Vehicle references.
        self.speed = self.declare_parameter("speed", 1.0).value
        self.lookahead_distance = self.declare_parameter(
            "lookahead_distance", 1.0
        ).value

        # Orientation controller gains.
        self.kp = self.declare_parameter("kp", 1.0).value
        self.ki = self.declare_parameter("ki", 0.0).value
        self.orientation_integral_limit = abs(
            self.declare_parameter("orientation_integral_limit", 1.0).value
        )

        # Previous odometry timestamp, used to estimate the sampling period.
        self.previous_time = None

        # Accumulated orientation error used by the integral term.
        self.orientation_error_integral = 0.0

        # Centerline path received by the node.
        self.path = None

        # ------------------------------------------------------------------
        # 2. ROS interfaces
        # ------------------------------------------------------------------
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

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
        if self.path is None:
            return

        # ------------------------------------------------------------------
        # 1. Sampling period.
        # ------------------------------------------------------------------
        stamp = odom.header.stamp
        current_time = stamp.sec + stamp.nanosec * 1e-9

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = current_time - self.previous_time

        self.previous_time = current_time

        # ------------------------------------------------------------------
        # 2. Vehicle pose.
        # ------------------------------------------------------------------
        pose = odom.pose.pose
        vehicle_x = pose.position.x
        vehicle_y = pose.position.y

        # Yaw is obtained from the quaternion orientation.
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)

        # ------------------------------------------------------------------
        # 3. Closest centerline point.
        # ------------------------------------------------------------------
        closest_index = 0
        closest_distance = math.inf

        for index, path_pose in enumerate(self.path.poses):
            point = path_pose.pose.position
            distance = math.hypot(point.x - vehicle_x, point.y - vehicle_y)

            if distance < closest_distance:
                closest_distance = distance
                closest_index = index

        # ------------------------------------------------------------------
        # 4. Lookahead target.
        # ------------------------------------------------------------------
        path_length = len(self.path.poses)
        target_index = closest_index

        for steps_ahead in range(path_length):
            index = (closest_index + steps_ahead) % path_length

            point = self.path.poses[index].pose.position
            distance = math.hypot(point.x - vehicle_x, point.y - vehicle_y)

            if distance >= self.lookahead_distance:
                target_index = index
                break

        target = self.path.poses[target_index].pose.position

        # ------------------------------------------------------------------
        # 5. Control error.
        # ------------------------------------------------------------------
        # The target vector is expressed in the global frame.
        dx = target.x - vehicle_x
        dy = target.y - vehicle_y

        # The desired yaw is the orientation of the vector that joins the
        # vehicle position with the lookahead target.
        desired_yaw = math.atan2(dy, dx)

        # oe is the orientation error between the vehicle yaw and the desired
        # yaw. The atan2 expression normalizes the error to [-pi, pi].
        oe = math.atan2(
            math.sin(desired_yaw - vehicle_yaw),
            math.cos(desired_yaw - vehicle_yaw),
        )

        # ------------------------------------------------------------------
        # 6. Steering command.
        # ------------------------------------------------------------------
        # A PI controller uses the accumulated orientation error, not only
        # the error from the current sampling period. Invalid or repeated
        # timestamps are not integrated.
        if math.isfinite(dt) and dt > 0.0:
            self.orientation_error_integral += oe * dt

            # Clamp the accumulator to avoid integral windup.
            self.orientation_error_integral = max(
                -self.orientation_integral_limit,
                min(
                    self.orientation_integral_limit,
                    self.orientation_error_integral,
                ),
            )

        steering = (
            oe * self.kp
            + self.orientation_error_integral * self.ki
        )

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
    node = CenterlineTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
