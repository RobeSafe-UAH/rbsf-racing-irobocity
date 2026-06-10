import math
from dataclasses import dataclass, field
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


# ── Configuración ────────────────────────────────────────────────────────────
TIMER_PERIOD = 0.05  # s → 20 Hz

@dataclass
class Move:
    distance:        float        # metros a avanzar
    speed:           float        # m/s
    steering_angle:  float = 0.0  # rad (+ izq, - der)

SEQUENCE: list[Move] = [
    Move(distance=2.0, speed=0.7),
    # Move(distance=0.5, speed=0.3, steering_angle=math.radians(20)),
    # Move(distance=1.0, speed=0.5, steering_angle=math.radians(-15)),
]

# ── Estado ───────────────────────────────────────────────────────────────────
class State(Enum):
    MOVING = auto()
    DONE   = auto()

# ── Nodo ─────────────────────────────────────────────────────────────────────
class MoveSequenceNode(Node):

    def __init__(self, moves: list[Move]) -> None:
        super().__init__('move_sequence')

        self.moves    = moves
        self.move_idx = 0

        self.pub   = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self._tick)

        self._start_step()

    # ── Pasos ────────────────────────────────────────────────────────────────
    def _start_step(self) -> None:
        move = self.moves[self.move_idx]
        self.step_duration = move.distance / move.speed
        self.step_start    = self.get_clock().now()
        self.state         = State.MOVING

        self.get_logger().info(
            f"[{self.move_idx + 1}/{len(self.moves)}] "
            f"dist={move.distance} m  speed={move.speed} m/s  "
            f"steering={math.degrees(move.steering_angle):.1f}°  "
            f"→ {self.step_duration:.2f} s"
        )

    def _next_step(self) -> None:
        self._drive(speed=0.0, steering_angle=0.0)
        self.move_idx += 1

        if self.move_idx < len(self.moves):
            self._start_step()
        else:
            self.state = State.DONE
            self.get_logger().info("Secuencia completada.")

    # ── Tick ─────────────────────────────────────────────────────────────────
    def _tick(self) -> None:
        if self.state == State.DONE:
            return

        move = self.moves[self.move_idx]

        if self._elapsed() < self.step_duration:
            self._drive(speed=move.speed, steering_angle=move.steering_angle)
        else:
            self._next_step()

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.step_start).nanoseconds * 1e-9

    def _drive(self, speed: float, steering_angle: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed          = speed
        msg.drive.steering_angle = steering_angle
        self.pub.publish(msg)


# ── Entrypoint ───────────────────────────────────────────────────────────────
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveSequenceNode(SEQUENCE)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()