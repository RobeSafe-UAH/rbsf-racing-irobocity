import sys
import termios

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from pynput import keyboard
from std_msgs.msg import Empty


BINDINGS = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    keyboard.Key.up: (1, 0),
    keyboard.Key.down: (-1, 0),
    keyboard.Key.left: (0, 1),
    keyboard.Key.right: (0, -1),
}


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self._teleop_enabled = self.declare_parameter('teleop_enabled', True).value
        self._forward_speed = self.declare_parameter('forward_speed', 0.8).value
        self._backward_speed = self.declare_parameter('backward_speed', 0.5).value
        self._steering_angle = self.declare_parameter('steering_angle', 0.4).value
        hz = self.declare_parameter('hz', 10).value
        output_topic = self.declare_parameter('output_topic', '/drive').value

        self._active = set()
        self._pub = self.create_publisher(AckermannDriveStamped, output_topic, 10)
        self._save_pub = self.create_publisher(Empty, "/save_map_trigger", 10)

        self._old_tty = None
        if sys.stdin.isatty():
            self._tty_fd = sys.stdin.fileno()
            self._old_tty = termios.tcgetattr(self._tty_fd)
            new = termios.tcgetattr(self._tty_fd)
            new[3] &= ~termios.ECHO
            termios.tcsetattr(self._tty_fd, termios.TCSANOW, new)

        if self._teleop_enabled:
            self.create_timer(1.0 / hz, self._publish)

        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

        if self._teleop_enabled:
            print(f'\nKeyboard teleop active  →  {output_topic}')
            print('  W / ↑   forward       S / ↓   backward')
            print('  A / ←   steer left    D / →   steer right')
        else:
            print('\nKeyboard map trigger active (drive disabled)')
        print('  m       save map')
        print('  q       quit\n')

    def _key_id(self, key):
        try:
            return key.char
        except AttributeError:
            return key

    def _on_press(self, key):
        kid = self._key_id(key)
        if kid == 'q':
            self.get_logger().info('Keyboard teleop stopped.')
            rclpy.shutdown()
            return
        if kid == 'm':
            self._save_pub.publish(Empty())
            return
        if self._teleop_enabled and kid in BINDINGS:
            self._active.add(kid)

    def _on_release(self, key):
        self._active.discard(self._key_id(key))

    def _publish(self):
        linear = 0.0
        angular = 0.0
        for k in list(self._active):
            if k in BINDINGS:
                l, a = BINDINGS[k]
                linear += l
                angular += a

        speed = linear * (self._forward_speed if linear > 0 else self._backward_speed)

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.speed = speed
        drive.drive.steering_angle = angular * self._steering_angle
        self._pub.publish(drive)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._old_tty is not None:
            termios.tcsetattr(node._tty_fd, termios.TCSANOW, node._old_tty)
        node._listener.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
