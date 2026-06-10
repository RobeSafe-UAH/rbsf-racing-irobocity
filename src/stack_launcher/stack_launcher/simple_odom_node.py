import csv
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy


# ── Config ────────────────────────────────────────────────────────────────────
TIMER_PERIOD   = 0.05  # s → 20 Hz
DEADMAN_BUTTON = 5     # RB — joy.buttons index for deadman switch
RELOAD_BUTTON  = 2     # X  — joy.buttons index to reload sequence

OUTPUT_DIR = Path("/ros2_ws/move_sequence")

L = 0.325  # wheelbase (m)


class OdomLogMode(Enum):
    LAST    = auto()
    ALL     = auto()
    EVERY_N = auto()

ODOM_LOG_MODE = OdomLogMode.ALL
ODOM_LOG_N    = 10


# ── Move dataclass ────────────────────────────────────────────────────────────
@dataclass
class Move:
    distance:       float   # arc length (m) — equals linear dist for straight moves
    speed:          float
    steering_angle: float = 0.0
    brake_margin:   float = 0.05

    @classmethod
    def straight(cls, distance: float, speed: float, brake_margin: float = 0.05) -> "Move":
        """Straight-line move."""
        return cls(distance=distance, speed=speed, steering_angle=0.0, brake_margin=brake_margin)

    @classmethod
    def arc(
        cls,
        radius: float,       # turning radius (m), always positive
        angle_deg: float,    # arc sweep (deg); positive = left, negative = right
        speed: float,
        brake_margin: float = 0.05,
    ) -> "Move":
        """
        Circular arc move.

        Ackermann steering relation: tan(δ) = L / r  →  δ = atan(L / r)
        Arc length: s = |r · θ|
        Sign of angle_deg determines turn direction (positive = left, negative = right).
        """
        angle_rad  = math.radians(angle_deg)
        arc_length = abs(radius * angle_rad)
        steering   = math.copysign(math.atan(L / radius), angle_deg)
        return cls(distance=arc_length, speed=speed, steering_angle=steering, brake_margin=brake_margin)


# ── Sequence ──────────────────────────────────────────────────────────────────
_steering = 0.3                          # rad
_r        = L / math.tan(_steering)     # ≈ 0.76 m

# # Straight line
# SEQUENCE: list[Move] = [
#     Move.straight(distance=2.0, speed=1.0, brake_margin=0.0),
#     Move.straight(distance=2.0, speed=0.0, brake_margin=0.0),
# ]

# Full oval sequence: straight → left arc → straight → right arc → stop
SEQUENCE: list[Move] = [
    Move.straight(distance=2.0, speed=1.0, brake_margin=0.0),
    Move.arc(radius=_r, angle_deg=180.0,  speed=0.7, brake_margin=0.0),   # quarter-circle left
    Move.straight(distance=2.0, speed=1.0, brake_margin=0.0),
    Move.arc(radius=_r, angle_deg=180.0, speed=0.7, brake_margin=0.0),    # second quarter-circle left
    Move.straight(distance=2.0, speed=0.0, brake_margin=0.0),  # virtual stop
]


# ── CSV columns ───────────────────────────────────────────────────────────────
CSV_HEADER = [
    "t", "move_idx",
    "pos_x", "pos_y", "pos_z",
    "vel_x", "vel_y", "vel_z",
    "target_speed", "target_steering_deg",
    "drive_speed",  "drive_steering_deg",
]


# ── State ─────────────────────────────────────────────────────────────────────
class State(Enum):
    WAITING = auto()
    MOVING  = auto()
    DONE    = auto()
    ABORTED = auto()


# ── Node ──────────────────────────────────────────────────────────────────────
class MoveSequenceNode(Node):

    def __init__(self, moves: list[Move]) -> None:
        super().__init__('move_sequence')

        self.moves = moves

        self._deadman_active: bool = False
        self._drive_topic_speed:    float = 0.0
        self._drive_topic_steering: float = 0.0

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        self.pub       = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.sub_odom  = self.create_subscription(Odometry, '/odom', self._odom_cb,  10)
        self.sub_drive = self.create_subscription(AckermannDriveStamped, '/drive', self._drive_cb, 10)
        self.sub_joy   = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.timer     = self.create_timer(TIMER_PERIOD, self._tick)

        # Initialize all run-specific state (also called on reload)
        self._reset(initial=True)

    # ── Reset ─────────────────────────────────────────────────────────────────

    def _reset(self, initial: bool = False) -> None:
        """
        Reset all per-run state and return to WAITING.
        Called on __init__ and whenever the reload button is pressed.
        Generates new csv/plot paths so each run gets its own files.
        """
        # Save plot from previous run before wiping records
        if not initial and self._records:
            self._close_csv()
            self._save_plot()

        self.move_idx    = 0
        self.state       = State.WAITING
        self.current_pos = None
        self.start_pos   = None
        self.last_odom   = None

        self._odom_count    = 0
        self._last_drive    = (0.0, 0.0)
        self._t0            = None

        # Incremental arc-length accumulator — replaces chord-based distance.
        # Summing small Euclidean steps converges to true arc length regardless
        # of path curvature, unlike a single chord from start to current pos.
        self._traveled_dist: float       = 0.0
        self._prev_pos:      tuple | None = None

        # New timestamped paths for this run
        ts = int(time.time())
        self._csv_path  = OUTPUT_DIR / f"odom_{ts}.csv"
        self._plot_path = OUTPUT_DIR / f"plot_{ts}.png"
        self._csv_file   = None
        self._csv_writer = None
        self._records:  list[dict] = []

        self.get_logger().info(
            f"{'Init' if initial else 'Reload'} — "
            f"hold button {DEADMAN_BUTTON} (RB) to start.  "
            f"Press button {RELOAD_BUTTON} (X) to reload after finish/abort.  "
            f"Odom log mode: {ODOM_LOG_MODE.name}"
            + (f" (N={ODOM_LOG_N})" if ODOM_LOG_MODE is OdomLogMode.EVERY_N else "")
        )

    # ── CSV helpers ───────────────────────────────────────────────────────────

    def _open_csv(self) -> None:
        self._csv_file   = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_HEADER)
        self._csv_writer.writeheader()
        self.get_logger().info(f"Recording to {self._csv_path}")

    def _write_csv_row(self, msg: Odometry) -> None:
        if self._csv_writer is None or self._t0 is None:
            return

        pos   = msg.pose.pose.position
        twist = msg.twist.twist.linear
        t_s, t_ns = self.get_clock().now().seconds_nanoseconds()
        t_elapsed  = (t_s + t_ns * 1e-9) - self._t0

        target_speed, target_steering = self._last_drive
        row = {
            "t":                   round(t_elapsed, 4),
            "move_idx":            self.move_idx,
            "pos_x":               round(pos.x,   4),
            "pos_y":               round(pos.y,   4),
            "pos_z":               round(pos.z,   4),
            "vel_x":               round(twist.x, 4),
            "vel_y":               round(twist.y, 4),
            "vel_z":               round(twist.z, 4),
            "target_speed":        round(target_speed,                              4),
            "target_steering_deg": round(math.degrees(target_steering),             4),
            "drive_speed":         round(self._drive_topic_speed,                   4),
            "drive_steering_deg":  round(math.degrees(self._drive_topic_steering),  4),
        }
        self._csv_writer.writerow(row)
        self._records.append(row)

    def _close_csv(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None
            self.get_logger().info(f"CSV saved → {self._csv_path}")

    # ── Plot ──────────────────────────────────────────────────────────────────

    def _save_plot(self) -> None:
        if not self._records:
            self.get_logger().warn("No data recorded — skipping plot.")
            return

        t            = [r["t"]            for r in self._records]
        vel_x        = [r["vel_x"]        for r in self._records]
        target_speed = [r["target_speed"] for r in self._records]
        drive_speed  = [r["drive_speed"]  for r in self._records]
        pos_x        = [r["pos_x"]        for r in self._records]
        pos_y        = [r["pos_y"]        for r in self._records]

        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle("Move Sequence — Odometry Recording", fontsize=13)

        ax = axes[0]
        ax.plot(t, vel_x,        label="odom vel_x (m/s)",  linewidth=1.5)
        ax.plot(t, drive_speed,  label="/drive speed (m/s)", linewidth=1.5, linestyle="-.")
        ax.step(t, target_speed, label="target speed (m/s)", linewidth=1.5, linestyle="--", where="post")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.set_title("Velocity vs Time")
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(pos_x, pos_y, linewidth=1.5, label="trajectory")
        ax.scatter(pos_x[0],  pos_y[0],  color="green", zorder=5, label="start")
        ax.scatter(pos_x[-1], pos_y[-1], color="red",   zorder=5, label="end")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("XY Trajectory")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")

        plt.tight_layout()
        plt.savefig(self._plot_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f"Plot saved  → {self._plot_path}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        pos     = msg.pose.pose.position
        new_pos = (pos.x, pos.y)

        # Accumulate incremental arc length while moving.
        # Each step is a tiny chord — at 20 Hz the car barely moves between
        # callbacks, so chord ≈ arc. Summing them converges to true path length
        # for any curvature, unlike measuring the single chord from start.
        if self.state is State.MOVING and self._prev_pos is not None:
            dx = new_pos[0] - self._prev_pos[0]
            dy = new_pos[1] - self._prev_pos[1]
            self._traveled_dist += math.hypot(dx, dy)

        self._prev_pos   = new_pos
        self.current_pos = new_pos
        self.last_odom   = msg
        self._odom_count += 1

        if self.start_pos is None:
            self.start_pos = self.current_pos
            self.get_logger().info(f"start_pos fixed: {self.start_pos}")

        if self.state is State.MOVING:
            self._write_csv_row(msg)

            if ODOM_LOG_MODE is OdomLogMode.ALL:
                self._log_odom(msg, label=f"odom #{self._odom_count}")
            elif ODOM_LOG_MODE is OdomLogMode.EVERY_N and self._odom_count % ODOM_LOG_N == 0:
                self._log_odom(msg, label=f"odom #{self._odom_count}")

    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        self._drive_topic_speed    = msg.drive.speed
        self._drive_topic_steering = msg.drive.steering_angle

    def _joy_cb(self, msg: Joy) -> None:
        # Deadman
        if DEADMAN_BUTTON < len(msg.buttons):
            self._deadman_active = bool(msg.buttons[DEADMAN_BUTTON])
        else:
            self.get_logger().warn(
                f"Deadman button {DEADMAN_BUTTON} out of range "
                f"(joy has {len(msg.buttons)} buttons)",
                throttle_duration_sec=2.0,
            )
            self._deadman_active = False

        # Reload — only on rising edge, only from terminal states
        if RELOAD_BUTTON < len(msg.buttons) and bool(msg.buttons[RELOAD_BUTTON]):
            if self.state in (State.DONE, State.ABORTED):
                self.get_logger().info("Reload button pressed — resetting sequence.")
                self._reset()

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log_odom(self, msg: Odometry, label: str = "odom") -> None:
        pos   = msg.pose.pose.position
        twist = msg.twist.twist.linear
        self.get_logger().info(
            f"[{label}] "
            f"pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m  "
            f"vel=({twist.x:.3f}, {twist.y:.3f}, {twist.z:.3f}) m/s  "
            f"drive={self._drive_topic_speed:.3f} m/s"
        )

    def _log_last_odom(self, context: str = "last") -> None:
        if self.last_odom is None:
            self.get_logger().warn("No odom received yet.")
            return
        self._log_odom(self.last_odom, label=f"{context} | odom #{self._odom_count}")

    def _log_drive(self, context: str = "") -> None:
        speed, steering = self._last_drive
        self.get_logger().info(
            f"[drive{' | ' + context if context else ''}] "
            f"speed={speed:.3f} m/s  steering={math.degrees(steering):.2f}°"
        )

    def _log_step(self) -> None:
        move = self.moves[self.move_idx]
        self.get_logger().info(
            f"[{self.move_idx + 1}/{len(self.moves)}] "
            f"dist={move.distance:.3f} m  speed={move.speed} m/s  "
            f"steering={math.degrees(move.steering_angle):.1f}°"
        )

    # ── Steps ─────────────────────────────────────────────────────────────────

    def _next_step(self) -> None:
        self._drive(speed=0.0, steering_angle=0.0)
        self._log_last_odom(context=f"move {self.move_idx + 1} end")
        self._log_drive(context=f"move {self.move_idx + 1} end")

        self.move_idx       += 1
        self.start_pos       = None
        self.current_pos     = None
        self._odom_count     = 0
        self._traveled_dist  = 0.0   # reset arc-length accumulator for next move
        self._prev_pos       = None  # reset delta reference for next move

        if self.move_idx < len(self.moves):
            self.state = State.MOVING
            self._log_step()
        else:
            self._finish()

    def _finish(self) -> None:
        self.state = State.DONE
        self.get_logger().info("Sequence completed. Press X to reload.")
        self._close_csv()
        self._save_plot()

    def _abort(self) -> None:
        self._drive(speed=0.0, steering_angle=0.0)
        self._log_last_odom(context="abort")
        self._log_drive(context="abort")
        self.state = State.ABORTED
        self._close_csv()
        self._save_plot()
        self.get_logger().warn("Deadman released — ABORTED. Press X to reload.")

    # ── Tick ──────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        match self.state:

            case State.WAITING:
                if self._deadman_active:
                    t_s, t_ns = self.get_clock().now().seconds_nanoseconds()
                    self._t0  = t_s + t_ns * 1e-9
                    self._open_csv()
                    self.state = State.MOVING
                    self.get_logger().info("Deadman active — starting sequence.")
                    self._log_step()

            case State.MOVING:
                if not self._deadman_active:
                    self._abort()
                    return

                if self.current_pos is None or self.start_pos is None:
                    move = self.moves[self.move_idx]
                    self._drive(speed=move.speed, steering_angle=move.steering_angle)
                    return

                move             = self.moves[self.move_idx]
                traveled         = self._distance_traveled()
                effective_target = move.distance - move.brake_margin

                if traveled < effective_target:
                    self._drive(speed=move.speed, steering_angle=move.steering_angle)
                else:
                    self.get_logger().info(
                        f"Braking at {traveled:.3f} m "
                        f"(target {move.distance:.3f} m, margin {move.brake_margin} m)"
                    )
                    self._next_step()

            case State.DONE | State.ABORTED:
                pass  # waiting for reload button

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _distance_traveled(self) -> float:
        """Return accumulated arc length since the current move started."""
        return self._traveled_dist

    def _drive(self, speed: float, steering_angle: float) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = 'base_link'
        msg.drive.speed          = speed
        msg.drive.steering_angle = steering_angle
        self.pub.publish(msg)
        self._last_drive = (speed, steering_angle)


# ── Entrypoint ────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveSequenceNode(SEQUENCE)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()