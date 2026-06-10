"""
AckermannToTwist node: converts AckermannDriveStamped to Twist.

Why this node exists
--------------------
The planning/control stack always publishes AckermannDriveStamped on ``/drive``
because that is the native interface of the real car (F1TENTH hardware).
The MVSim simulator, however, expects a geometry_msgs/Twist on
``/robot1/cmd_vel``.  This node bridges the two so that the same stack can
run unmodified against both the physical vehicle and the simulator — only the
launch configuration changes which topics are remapped.

Ackermann kinematics
--------------------
    linear.x  = speed
    angular.z = speed * tan(steering_angle) / wheelbase
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node


class AckermannToTwistNode(Node):
    """Converts AckermannDriveStamped commands to Twist messages."""

    def __init__(self) -> None:
        super().__init__("ackermann_to_twist")

        self.declare_parameter(
            "wheelbase",
            0.33,
            ParameterDescriptor(description="Distance between front and rear axles [m]"),
        )
        self.declare_parameter(
            "max_steering_angle",
            0.5,
            ParameterDescriptor(description="Maximum steering angle [rad], used for clamping"),
        )
        self.declare_parameter(
            "input_topic",
            "drive",
            ParameterDescriptor(description="AckermannDriveStamped input topic"),
        )
        self.declare_parameter(
            "output_topic",
            "cmd_vel",
            ParameterDescriptor(description="Twist output topic"),
        )

        self.wheelbase: float = self.get_parameter("wheelbase").value
        self.max_steering_angle: float = self.get_parameter("max_steering_angle").value
        self.input_topic: str = self.get_parameter("input_topic").value
        self.output_topic: str = self.get_parameter("output_topic").value

        self._pub = self.create_publisher(Twist, self.output_topic, 10)
        self._sub = self.create_subscription(
            AckermannDriveStamped,
            self.input_topic,
            self._on_ackermann_msg,
            10,
        )

        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info(
            f"AckermannToTwist ready | "
            f"wheelbase={self.wheelbase:.3f} m | "
            f"max_steering={math.degrees(self.max_steering_angle):.1f} deg"
        )

    # ------------------------------------------------------------------
    # Parameter update callback
    # ------------------------------------------------------------------

    def _on_params_changed(self, params) -> SetParametersResult:
        for p in params:
            if p.name in ("wheelbase", "max_steering_angle", "input_topic", "output_topic"):
                self.wheelbase = self.get_parameter("wheelbase").value
                self.max_steering_angle = self.get_parameter("max_steering_angle").value
                self.input_topic = self.get_parameter("input_topic").value
                self.output_topic = self.get_parameter("output_topic").value
                break
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------

    def _on_ackermann_msg(self, msg: AckermannDriveStamped) -> None:
        twist = self._ackermann_to_twist(msg.drive.speed, msg.drive.steering_angle)
        self._pub.publish(twist)

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def _ackermann_to_twist(self, speed: float, steering_angle: float) -> Twist:
        """
        Convert Ackermann drive to Twist using bicycle model.

        Args:
            speed: Longitudinal speed [m/s].
            steering_angle: Front-wheel steering angle [rad].

        Returns:
            Twist with linear.x and angular.z populated.
        """
        # Clamp steering to physical limits
        steering_angle = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering_angle),
        )

        # angular.z = v * tan(delta) / L
        angular_z = speed * math.tan(steering_angle) / self.wheelbase

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = angular_z
        return twist


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannToTwistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
