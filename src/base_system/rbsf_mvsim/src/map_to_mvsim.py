#!/usr/bin/env python3
"""
map_to_mvsim.py
================
Convert a ROS occupancy-grid map (PNG + YAML) into an MVSim .world.xml file.

The tool:
  1. Reads the YAML metadata (resolution, origin, thresholds).
  2. Thresholds the PNG to a binary occupied/free map.
  3. Extracts wall contours and simplifies them with Douglas-Peucker.
  4. Emits <vertical_plane> elements for every segment and a <horizontal_plane>
     ground tile covering the full map extent.

Usage
-----
    python map_to_mvsim.py map.yaml [OPTIONS]

    # São Paulo example
    python map_to_mvsim.py SaoPaulo_map.yaml \\
        --epsilon 0.15 \\
        --wall-height 0.3 \\
        --vehicle f1tenth \\
        --preview

Dependencies
------------
    pip install opencv-python numpy pyyaml
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml


# ─────────────────────────────────────────────────────────────
# Config / YAML
# ─────────────────────────────────────────────────────────────

def load_yaml(yaml_path: Path) -> dict:
    with open(yaml_path) as f:
        return yaml.safe_load(f)


def resolve_image(yaml_path: Path, image_field: str) -> Path:
    """Image path in YAML can be relative to the YAML file."""
    p = Path(image_field)
    return p if p.is_absolute() else yaml_path.parent / p


# ─────────────────────────────────────────────────────────────
# Image loading & thresholding
# ─────────────────────────────────────────────────────────────

def load_binary_map(
    image_path: Path,
    negate: bool,
    occupied_thresh: float,
) -> np.ndarray:
    """
    Return a uint8 binary image: 255 = wall / occupied, 0 = free.

    ROS map_server convention (negate=0):
        occupancy = (255 - pixel) / 255   →  white=free, black=occupied
    With negate=1 the formula is inverted.
    """
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"[ERROR] Cannot read image: {image_path}")

    img_f = img.astype(np.float32) / 255.0
    occupancy = img_f if negate else (1.0 - img_f)

    binary = ((occupancy > occupied_thresh) * 255).astype(np.uint8)
    return binary


# ─────────────────────────────────────────────────────────────
# Coordinate transform  (pixel → world)
# ─────────────────────────────────────────────────────────────

def px_to_world(
    col: float,
    row: float,
    img_h: int,
    resolution: float,
    origin: list[float],
) -> tuple[float, float]:
    """
    Convert pixel (col, row) to world (x, y).

    map_server places `origin` at the bottom-left corner of the image.
    Row 0 is the TOP of the image, so y is flipped.
    """
    wx = origin[0] + col * resolution
    wy = origin[1] + (img_h - row) * resolution
    return wx, wy


# ─────────────────────────────────────────────────────────────
# Spawn pose auto-detection
# ─────────────────────────────────────────────────────────────

def find_spawn_pose(
    binary_map: np.ndarray,
    resolution: float,
    origin: list[float],
    min_clearance_m: float = 0.15,
    max_clearance_m: float = 2.5,
) -> tuple[float, float]:
    """
    Return a world (x, y) spawn point on the track centreline.

    A point equidistant from the two nearest walls is, by definition, a local
    maximum of the distance transform — the medial axis of the free space.
    This is the track centreline.

    Strategy:
      1. Distance transform → clearance per free pixel.
      2. Medial axis = pixels that are local maxima of the distance transform
         (no neighbour has strictly greater distance).
      3. Estimate track half-width from the 25th percentile of all non-zero
         clearance values (robust to large open infields that skew the max).
      4. Keep medial-axis pixels in [min_clearance, estimated_halfwidth * 1.5].
      5. Pick the one closest to the centroid of those candidates.
      6. Fallback: widen the window progressively if nothing survives.
    """
    free_mask = (binary_map == 0).astype(np.uint8)
    dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, maskSize=5)

    # ── Medial axis: local maxima of the distance transform ──
    # A pixel is on the medial axis if it is >= all 8 neighbours.
    kernel = np.ones((3, 3), dtype=np.uint8)
    dist_dilated = cv2.dilate(dist, kernel)
    medial_axis = (dist >= dist_dilated) & (free_mask == 1)

    # ── Estimate track half-width ────────────────────────────
    nonzero_vals = dist[dist > 0]
    if nonzero_vals.size > 0:
        estimated_hw = float(np.percentile(nonzero_vals, 25)) * resolution
        adaptive_max = min(estimated_hw * 1.5, max_clearance_m)
    else:
        adaptive_max = max_clearance_m

    print(
        f"[map_to_mvsim] Spawn search: medial axis, clearance window "
        f"[{min_clearance_m:.2f}, {adaptive_max:.2f}] m"
    )

    def _best_on_medial_axis(lo_px: float, hi_px: float) -> tuple[int, int] | None:
        valid = medial_axis & (dist >= lo_px) & (dist <= hi_px)
        if not np.any(valid):
            return None
        rows_v, cols_v = np.where(valid)
        # Anchor = centroid of the valid medial-axis pixels
        cy, cx = rows_v.mean(), cols_v.mean()
        d_anchor = np.hypot(cols_v - cx, rows_v - cy)
        best = int(np.argmin(d_anchor))
        return int(rows_v[best]), int(cols_v[best])

    lo_px = min_clearance_m / resolution
    hi_px = adaptive_max / resolution

    result = _best_on_medial_axis(lo_px, hi_px)

    if result is None:
        for scale in [2.0, 4.0, 8.0]:
            result = _best_on_medial_axis(lo_px, hi_px * scale)
            if result is not None:
                break

    if result is None:
        # Last resort: highest clearance point anywhere
        _, _, _, max_loc = cv2.minMaxLoc(dist)
        best_row, best_col = int(max_loc[1]), int(max_loc[0])
    else:
        best_row, best_col = result

    clearance_m = float(dist[best_row, best_col]) * resolution
    wx, wy = px_to_world(best_col, best_row, binary_map.shape[0], resolution, origin)

    yaw_deg = _medial_axis_yaw(medial_axis, best_row, best_col, resolution)

    print(
        f"[map_to_mvsim] Auto spawn : ({wx:.3f}, {wy:.3f}) m  "
        f"| clearance ≈ {clearance_m:.2f} m  "
        f"| yaw ≈ {yaw_deg:.1f}°  "
        f"(override with --init-pose x y yaw)"
    )
    return wx, wy, yaw_deg


def _medial_axis_yaw(
    medial_axis: np.ndarray,
    row: int,
    col: int,
    resolution: float,
    radius_m: float = 1.5,
) -> float:
    """
    Estimate track orientation at (row, col) by fitting a line to the nearby
    medial-axis pixels using PCA.

    The principal eigenvector of the local point cloud is the tangent to the
    track centreline. Converting it to world yaw accounts for the image→world
    y-flip (row increases downward in image, y increases upward in world).

    Returns yaw in degrees (MVSim convention).
    """
    radius_px = max(3, int(radius_m / resolution))

    # Crop a square neighbourhood around the spawn pixel
    h, w = medial_axis.shape
    r0, r1 = max(0, row - radius_px), min(h, row + radius_px + 1)
    c0, c1 = max(0, col - radius_px), min(w, col + radius_px + 1)
    patch = medial_axis[r0:r1, c0:c1]

    rows_local, cols_local = np.where(patch)
    if len(rows_local) < 2:
        return 0.0  # not enough points — default forward

    # Centre the point cloud
    pts = np.stack([cols_local, rows_local], axis=1).astype(np.float32)
    pts -= pts.mean(axis=0)

    # PCA via SVD: first right-singular vector = principal direction in (col, row)
    _, _, vt = np.linalg.svd(pts, full_matrices=False)
    dcol, drow = float(vt[0, 0]), float(vt[0, 1])

    # Convert image tangent (dcol, drow) → world tangent (dx, dy)
    # World y is flipped relative to image row, so dy = -drow
    dx, dy = dcol, -drow

    yaw_rad = np.arctan2(dy, dx)
    return float(np.degrees(yaw_rad))


# ─────────────────────────────────────────────────────────────
# Contour extraction & simplification
# ─────────────────────────────────────────────────────────────

def extract_wall_segments(
    binary_map: np.ndarray,
    resolution: float,
    origin: list[float],
    epsilon_m: float,
    min_perimeter_m: float,
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """
    Detect contours of occupied regions and approximate them as polylines.

    Returns
    -------
    List of ((x0, y0), (x1, y1)) segment pairs in world coordinates.
    """
    img_h = binary_map.shape[0]
    epsilon_px = max(1.0, epsilon_m / resolution)
    min_perimeter_px = min_perimeter_m / resolution

    contours, _ = cv2.findContours(
        binary_map, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )

    segments: list[tuple] = []
    for contour in contours:
        if cv2.arcLength(contour, closed=True) < min_perimeter_px:
            continue

        approx = cv2.approxPolyDP(contour, epsilon_px, closed=True)
        pts = approx.reshape(-1, 2)  # (N, 2)  col, row

        n = len(pts)
        for i in range(n):
            col0, row0 = pts[i]
            col1, row1 = pts[(i + 1) % n]
            p0 = px_to_world(col0, row0, img_h, resolution, origin)
            p1 = px_to_world(col1, row1, img_h, resolution, origin)
            # Skip degenerate zero-length segments
            if p0 != p1:
                segments.append((p0, p1))

    return segments


# ─────────────────────────────────────────────────────────────
# XML generation
# ─────────────────────────────────────────────────────────────

def _pretty_indent(elem: ET.Element, level: int = 0) -> None:
    """In-place pretty-print indentation for ElementTree."""
    pad = "\n" + "    " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "    "
        for child in elem:
            _pretty_indent(child, level + 1)
        if not child.tail or not child.tail.strip():  # type: ignore[possibly-undefined]
            child.tail = pad
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def _sub(parent: ET.Element, tag: str, text: str = "", **attribs) -> ET.Element:
    el = ET.SubElement(parent, tag, **attribs)
    if text:
        el.text = text
    return el


def build_mvsim_xml(
    segments: list,
    img_shape: tuple[int, int],
    resolution: float,
    origin: list[float],
    *,
    wall_height: float,
    wall_color: str,
    ground_color: str,
    cam_distance: float,
    init_pose: tuple[float, float, float],
) -> str:
    img_h, img_w = img_shape

    # World bounding box in metres
    x_min = origin[0]
    y_min = origin[1]
    x_max = origin[0] + img_w * resolution
    y_max = origin[1] + img_h * resolution

    root = ET.Element("mvsim_world", version="1.0")
    _sub(root, "simul_timestep", "0")

    gui = _sub(root, "gui")
    _sub(gui, "ortho", "true")
    _sub(gui, "show_forces", "false")
    _sub(gui, "cam_distance", str(cam_distance))
    _sub(gui, "fov_deg", "60")
    _sub(gui, "refresh_fps", "60")

    _sub(root, "variable", name="WALL_H", value=str(wall_height))

    # Ground plane
    gnd = _sub(root, "element", **{"class": "horizontal_plane"})
    _sub(gnd, "cull_face", "BACK")
    _sub(gnd, "x_min", f"{x_min:.4f}")
    _sub(gnd, "y_min", f"{y_min:.4f}")
    _sub(gnd, "x_max", f"{x_max:.4f}")
    _sub(gnd, "y_max", f"{y_max:.4f}")
    _sub(gnd, "z", "0.0")
    _sub(gnd, "color", ground_color)

    # Wall segments
    for (x0, y0), (x1, y1) in segments:
        wall = _sub(root, "element", **{"class": "vertical_plane"})
        _sub(wall, "cull_face", "NONE")
        _sub(wall, "x0", f"{x0:.4f}")
        _sub(wall, "y0", f"{y0:.4f}")
        _sub(wall, "x1", f"{x1:.4f}")
        _sub(wall, "y1", f"{y1:.4f}")
        _sub(wall, "z", "0.0")
        _sub(wall, "height", "$f{WALL_H}")
        _sub(wall, "color", wall_color)

    # Vehicle — Jackal with 2D lidar raytrace
    x, y, yaw = init_pose
    _sub(
        root, "include",
        file="/opt/ros/humble/share/mvsim/definitions/jackal.vehicle.xml",
        default_sensors="true",
        lidar2d_raytrace="true",
    )
    veh = _sub(root, "vehicle", name="r1", **{"class": "jackal"})
    _sub(veh, "init_pose", f"{x:.4f} {y:.4f} {yaw:.1f}")

    _pretty_indent(root)
    body = ET.tostring(root, encoding="unicode")
    return '<?xml version="1.0" encoding="utf-8"?>\n' + body


# ─────────────────────────────────────────────────────────────
# Debug preview
# ─────────────────────────────────────────────────────────────

def show_preview(
    binary_map: np.ndarray,
    segments: list,
    resolution: float,
    origin: list[float],
) -> None:
    preview = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
    img_h = binary_map.shape[0]

    for (x0, y0), (x1, y1) in segments:
        col0 = int((x0 - origin[0]) / resolution)
        row0 = int(img_h - (y0 - origin[1]) / resolution)
        col1 = int((x1 - origin[0]) / resolution)
        row1 = int(img_h - (y1 - origin[1]) / resolution)
        cv2.line(preview, (col0, row0), (col1, row1), (0, 200, 0), 1)

    # Rescale for display if the image is very large
    max_dim = 900
    h, w = preview.shape[:2]
    if max(h, w) > max_dim:
        scale = max_dim / max(h, w)
        preview = cv2.resize(preview, (int(w * scale), int(h * scale)))

    cv2.imshow("Wall segments (green) — press any key to close", preview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ─────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────

VEHICLE_XML_MAP = {
    "jackal":            "/opt/ros/humble/share/mvsim/definitions/jackal.vehicle.xml",
    "ackermann":         "/opt/ros/humble/share/mvsim/definitions/ackermann_vehicle.vehicle.xml",
    "turtlebot3_burger": "/opt/ros/humble/share/mvsim/definitions/turtlebot3_burger.vehicle.xml",
    "turtlebot3_waffle": "/opt/ros/humble/share/mvsim/definitions/turtlebot3_waffle_pi.vehicle.xml",
}

# Aliases kept for backward compat / convenience
VEHICLE_ALIASES = {
    "f1tenth": "ackermann",
}


def resolve_vehicle(name: str) -> tuple[str, str]:
    """
    Return (vehicle_class, xml_path) for a given vehicle name or alias.
    Falls back gracefully if the resolved XML does not exist on disk.
    """
    canonical = VEHICLE_ALIASES.get(name, name)
    xml_path = VEHICLE_XML_MAP.get(canonical, "")

    # Runtime check — warn early instead of crashing MVSim
    if xml_path and not Path(xml_path).exists():
        available = _discover_vehicle_xmls()
        if available:
            suggestion = ", ".join(available[:5])
            print(
                f"[map_to_mvsim] WARNING: '{xml_path}' not found on disk.\n"
                f"[map_to_mvsim]   Available definitions: {suggestion}\n"
                f"[map_to_mvsim]   Use --vehicle-xml to specify the correct path."
            )
        else:
            print(
                f"[map_to_mvsim] WARNING: '{xml_path}' not found. "
                "Is mvsim installed? Use --vehicle-xml to override."
            )

    return canonical, xml_path


def _discover_vehicle_xmls() -> list[str]:
    """Return vehicle XML paths found in the mvsim ROS share directory."""
    share_dirs = [
        Path("/opt/ros/humble/share/mvsim/definitions"),
        Path("/opt/ros/jazzy/share/mvsim/definitions"),
    ]
    found = []
    for d in share_dirs:
        if d.is_dir():
            found.extend(str(p) for p in sorted(d.glob("*.vehicle.xml")))
    return found


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--list-vehicles", action="store_true",
        help="Print vehicle XML files found in the mvsim definitions directory and exit.",
    )
    p.add_argument("yaml", type=Path, nargs="?", help="Map YAML file path")
    p.add_argument(
        "-o", "--output", type=Path, default=None,
        help="Output .world.xml path (default: <map_stem>.world.xml)",
    )
    p.add_argument(
        "--epsilon", type=float, default=0.15,
        help="Douglas-Peucker tolerance in metres (default: 0.15)",
    )
    p.add_argument(
        "--min-perimeter", type=float, default=0.5,
        help="Minimum contour perimeter in metres to keep (default: 0.5)",
    )
    p.add_argument(
        "--wall-height", type=float, default=0.3,
        help="Wall height in metres (default: 0.3)",
    )
    p.add_argument(
        "--wall-color", default="#cc2200",
        help="Wall hex colour (default: #cc2200)",
    )
    p.add_argument(
        "--ground-color", default="#404040",
        help="Ground hex colour (default: #404040)",
    )
    p.add_argument(
        "--vehicle",
        choices=list(VEHICLE_XML_MAP.keys()) + list(VEHICLE_ALIASES.keys()) + ["none"],
        default="ackermann",
        help=(
            "Vehicle class (default: ackermann). "
            "Alias 'f1tenth' maps to 'ackermann'. Use 'none' to omit."
        ),
    )
    p.add_argument(
        "--init-pose", nargs=3, type=float, metavar=("X", "Y", "YAW"),
        default=None,
        help=(
            "Vehicle spawn pose in world coordinates: x y yaw_deg. "
            "If omitted, auto-detected as the free cell with maximum "
            "clearance from walls (track centerline candidate)."
        ),
    )
    p.add_argument(
        "--vehicle-xml", default=None,
        help="Override vehicle XML path (overrides --vehicle lookup)",
    )
    p.add_argument(
        "--cam-distance", type=float, default=60.0,
        help="Initial camera distance in metres (default: 60)",
    )
    p.add_argument(
        "--preview", action="store_true",
        help="Show a debug window with the extracted contours",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if args.list_vehicles:
        found = _discover_vehicle_xmls()
        if found:
            print("[map_to_mvsim] Available vehicle definitions:")
            for p in found:
                print(f"  {p}")
        else:
            print("[map_to_mvsim] No vehicle definitions found. Is mvsim installed?")
        sys.exit(0)

    if not args.yaml:
        sys.exit("[ERROR] Provide a YAML file or use --list-vehicles.")

    if not args.yaml.exists():
        sys.exit(f"[ERROR] YAML not found: {args.yaml}")

    # ── Load config ──────────────────────────────────────────
    cfg = load_yaml(args.yaml)
    resolution   = float(cfg["resolution"])
    origin       = list(cfg["origin"])          # [x, y, yaw]
    negate       = bool(int(cfg.get("negate", 0)))
    occ_thresh   = float(cfg.get("occupied_thresh", 0.65))
    image_path   = resolve_image(args.yaml, cfg["image"])

    print(f"[map_to_mvsim] Image   : {image_path}")
    print(f"[map_to_mvsim] Res     : {resolution} m/px")
    print(f"[map_to_mvsim] Origin  : {origin}")
    print(f"[map_to_mvsim] Negate  : {negate}  |  occ_thresh: {occ_thresh}")

    # ── Binary map ───────────────────────────────────────────
    binary_map = load_binary_map(image_path, negate, occ_thresh)
    h, w = binary_map.shape
    print(f"[map_to_mvsim] Map size: {w}×{h} px  "
          f"({w*resolution:.1f}×{h*resolution:.1f} m)")

    occupied_px = int(np.count_nonzero(binary_map))
    print(f"[map_to_mvsim] Occupied pixels: {occupied_px}")

    # ── Contours → segments ──────────────────────────────────
    print(f"[map_to_mvsim] Extracting contours (ε={args.epsilon} m, "
          f"min_perimeter={args.min_perimeter} m)...")
    segments = extract_wall_segments(
        binary_map, resolution, origin,
        epsilon_m=args.epsilon,
        min_perimeter_m=args.min_perimeter,
    )
    print(f"[map_to_mvsim] Wall segments: {len(segments)}")

    if args.preview:
        show_preview(binary_map, segments, resolution, origin)

    # ── Spawn pose ────────────────────────────────────────────
    if args.init_pose is not None:
        init_pose: tuple[float, float, float] = tuple(args.init_pose)  # type: ignore
        print(f"[map_to_mvsim] Manual spawn: {init_pose}")
    else:
        wx, wy, yaw_deg = find_spawn_pose(binary_map, resolution, origin)
        init_pose = (wx, wy, yaw_deg)

    # ── Build XML ─────────────────────────────────────────────
    xml_str = build_mvsim_xml(
        segments,
        img_shape=(h, w),
        resolution=resolution,
        origin=origin,
        wall_height=args.wall_height,
        wall_color=args.wall_color,
        ground_color=args.ground_color,
        cam_distance=args.cam_distance,
        init_pose=init_pose,
    )

    output = args.output or args.yaml.with_suffix("").with_suffix(".world.xml")
    output.write_text(xml_str, encoding="utf-8")
    print(f"[map_to_mvsim] Written : {output}")


if __name__ == "__main__":
    main()
