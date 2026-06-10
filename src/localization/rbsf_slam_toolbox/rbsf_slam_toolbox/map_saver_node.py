#!/usr/bin/env python3
"""Save a SLAM Toolbox map via joystick button press."""

import glob
import os
import shutil
import threading
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Joy
from slam_toolbox.srv import SaveMap, SerializePoseGraph


class MapSaverNode(Node):
    def __init__(self):
        super().__init__("map_saver_node")
        self.declare_parameter("map_output_dir", "/ros2_ws/maps")
        self.declare_parameter("save_button", 0)

        self._cb_group = ReentrantCallbackGroup()
        self._last_button = 0
        self._saving = False
        self._save_lock = threading.Lock()

        self._save_map_client = self.create_client(
            SaveMap, "/slam_toolbox/save_map", callback_group=self._cb_group
        )
        self._serialize_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map", callback_group=self._cb_group
        )

        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.get_logger().info(
            f"MapSaverNode ready — press button "
            f"{self.get_parameter('save_button').get_parameter_value().integer_value} "
            f"to save map."
        )

    def _joy_callback(self, msg: Joy) -> None:
        idx = self.get_parameter("save_button").get_parameter_value().integer_value
        if idx >= len(msg.buttons):
            return
        current = msg.buttons[idx]
        if current == 1 and self._last_button == 0:
            with self._save_lock:
                if self._saving:
                    self.get_logger().warn("Map save already in progress, ignoring.")
                else:
                    self._saving = True
                    threading.Thread(target=self._do_save, daemon=True).start()
        self._last_button = current

    def _do_save(self) -> None:
        try:
            output_dir = self.get_parameter("map_output_dir").get_parameter_value().string_value
            os.makedirs(output_dir, exist_ok=True)

            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_name = f"map_{stamp}"
            home_dir = os.path.expanduser("~")
            home_base = os.path.join(home_dir, base_name)
            self.get_logger().info(f"Saving map (staging at: {home_base})")

            if not self._call_save_map(home_base):
                self.get_logger().error("Failed to save occupancy map, aborting.")
                return
            self._call_serialize_pose_graph(home_base + "_graph")
            map_dir = os.path.join(output_dir, base_name)
            os.makedirs(map_dir, exist_ok=True)
            self._move_map_files(home_dir, base_name, map_dir)
        finally:
            with self._save_lock:
                self._saving = False

    def _wait_future(self, future, timeout_sec: float = 10.0) -> bool:
        # Poll from the save thread; the MultiThreadedExecutor running on the
        # main thread processes the response and marks the future done.
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.05)
        return True

    def _move_map_files(self, src_dir: str, base_name: str, dst_dir: str) -> None:
        matches = glob.glob(os.path.join(src_dir, base_name + "*"))
        if not matches:
            self.get_logger().error(
                f"No map files found in {src_dir} matching '{base_name}*'"
            )
            return
        for src in matches:
            dst = os.path.join(dst_dir, os.path.basename(src))
            shutil.move(src, dst)
            self.get_logger().info(f"Saved: {dst}")

    def _call_save_map(self, filepath: str) -> bool:
        """Save .pgm + .yaml map for Nav2."""
        if not self._save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("save_map service not available")
            return False
        request = SaveMap.Request()
        request.name.data = filepath
        future = self._save_map_client.call_async(request)
        if not self._wait_future(future):
            self.get_logger().error("save_map timed out")
            return False
        result = future.result()
        self.get_logger().info(f"save_map result: {result.result}")
        return result.result == SaveMap.Response.RESULT_SUCCESS

    def _call_serialize_pose_graph(self, filepath: str) -> bool:
        """Save pose-graph for lifelong/continued mapping."""
        if not self._serialize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("serialize_map service not available")
            return False
        request = SerializePoseGraph.Request()
        request.filename = filepath
        future = self._serialize_client.call_async(request)
        if not self._wait_future(future):
            self.get_logger().error("serialize_map timed out")
            return False
        result = future.result()
        self.get_logger().info(f"serialize_map result: {result.result}")
        return result.result == SerializePoseGraph.Response.RESULT_SUCCESS


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    node = MapSaverNode()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
