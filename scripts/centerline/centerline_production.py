"""
Production centerline extractor.

Extracts a smooth, closed centerline from an occupancy-grid map and exports
it as a CSV of world-frame waypoints with heading angle.  The output is
designed to be published directly as a nav_msgs/Path for cyclic following
(controller iterates with  idx % n_waypoints — no duplicate first point).

Algorithm (see centerline_student.py for the step-by-step derivation):
  1. Load map image + YAML metadata
  2. Auto-detect diagram-style PNGs and preprocess in-memory if needed
  3. Threshold white corridor pixels (> 240) → free-space mask
  4. Find outer and inner wall contours via RETR_CCOMP hierarchy
  5. Arc-length resample the outer wall at --spacing intervals
  6. Nearest-neighbour match outer → inner; midpoint = raw centerline
  7. Fit a periodic cubic spline through all raw midpoints
  8. Evaluate at n_out uniform arc-length positions (endpoint=False)
  9. Convert pixel → world frame; compute yaw from spline tangent
 10. Save CSV: x_meters, y_meters, yaw_rad

Output (created in the current working directory):
    {CIRCUIT_NAME}_{YYYY-MM-DD}/
        <original map image>     — copy of the source PNG/PGM
        <original map YAML>      — copy of the source YAML
        centerline_output.csv    — x_meters, y_meters, yaw_rad

Usage:
    uv run scripts/centerline/centerline_production.py \\
        --map  path/to/map.pgm  --yaml path/to/map.yaml \\
        [--output centerline_output.csv] [--spacing 0.10] [--plot]

    --output overrides only the CSV filename; the run folder and file copies
    are always created.
"""

import argparse
import os
import shutil
import sys
from datetime import date

import cv2
import numpy as np
import yaml
from scipy.interpolate import CubicSpline

# ── Default parameters ─────────────────────────────────────────────────────────

DEFAULT_SPACING = 0.10   # metres between output waypoints

# ── Map loading ────────────────────────────────────────────────────────────────

def _load(map_path: str, yaml_path: str):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot read map image: {map_path}")
    return meta, img


def _is_diagram_png(img: np.ndarray) -> bool:
    """True when white connected components touch the image border (diagram map)."""
    _, white = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    _, labels = cv2.connectedComponents(white)
    h, w = img.shape
    border = (
        set(labels[0, :].tolist()) | set(labels[-1, :].tolist()) |
        set(labels[:, 0].tolist()) | set(labels[:, -1].tolist())
    )
    border.discard(0)
    return len(border) > 0


def _preprocess_diagram(img: np.ndarray) -> np.ndarray:
    """
    Paint outer background and infield gray (127) so white = corridor only.

    Two passes:
      1. Outer background — white blobs that touch the image border.
      2. Infield          — top-level white blobs with no children in the
                            RETR_CCOMP hierarchy (the corridor ring always
                            has a child; the infield blob does not).
    """
    # Pass 1: outer background
    _, white = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    _, labels = cv2.connectedComponents(white)
    h, w = img.shape
    border = (
        set(labels[0, :].tolist()) | set(labels[-1, :].tolist()) |
        set(labels[:, 0].tolist()) | set(labels[:, -1].tolist())
    )
    border.discard(0)
    out = img.copy()
    for lbl in border:
        out[labels == lbl] = 127

    # Pass 2: infield
    _, walls = cv2.threshold(out, 80, 255, cv2.THRESH_BINARY_INV)
    close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    walls_closed = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, close_k)
    white_closed = np.where((out > 240) & (walls_closed == 0),
                            np.uint8(255), np.uint8(0))
    contours, hierarchy = cv2.findContours(
        white_closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )
    if hierarchy is not None and len(contours) > 0:
        h2 = hierarchy[0]
        for i, c in enumerate(contours):
            if h2[i][3] == -1 and h2[i][2] == -1:   # top-level, no children = infield
                mask = np.zeros(out.shape[:2], dtype=np.uint8)
                cv2.drawContours(mask, contours, i, 255, cv2.FILLED)
                out[(mask == 255) & (out > 80)] = 127
    return out

# ── Core pipeline ──────────────────────────────────────────────────────────────

def _build_free_mask(img: np.ndarray) -> np.ndarray:
    _, free_raw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.morphologyEx(free_raw, cv2.MORPH_OPEN, kernel)


def _find_walls(free_mask: np.ndarray):
    contours, hierarchy = cv2.findContours(
        free_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    if hierarchy is None or len(contours) < 2:
        raise RuntimeError(
            "Expected at least 2 contours — check the free-space mask.\n"
            "If using a diagram PNG, ensure it is preprocessed or run without --no-preprocess."
        )
    h = hierarchy[0]
    outer_contours = [c.reshape(-1, 2) for i, c in enumerate(contours) if h[i][3] == -1]
    hole_contours  = [c.reshape(-1, 2) for i, c in enumerate(contours) if h[i][3] != -1]
    if not hole_contours:
        raise RuntimeError("No inner wall found — the free-space mask may not be a closed ring.")
    outer = max(outer_contours, key=cv2.contourArea).astype(np.float64)
    inner = max(hole_contours,  key=cv2.contourArea).astype(np.float64)
    return outer, inner


def _resample(pts: np.ndarray, n: int) -> np.ndarray:
    """Resample a closed contour to n uniformly-spaced points by arc length."""
    pts = pts.astype(np.float64)
    diffs   = np.diff(pts, axis=0, append=pts[:1])
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    cum_len = np.concatenate([[0.0], np.cumsum(seg_len)])
    total   = cum_len[-1]
    x_ext   = np.append(pts[:, 0], pts[0, 0])
    y_ext   = np.append(pts[:, 1], pts[0, 1])
    t       = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(t, cum_len, x_ext),
                             np.interp(t, cum_len, y_ext)])


def _raw_midpoints(outer_r: np.ndarray, inner: np.ndarray) -> np.ndarray:
    """Nearest-neighbour match outer → inner; midpoint = raw centerline."""
    diffs = outer_r[:, None, :] - inner[None, :, :]        # (N, M, 2)
    idx   = np.argmin((diffs ** 2).sum(axis=2), axis=1)    # (N,)
    return (outer_r + inner[idx]) / 2.0


def _fit_spline(raw_cl: np.ndarray, n_out: int):
    """
    Fit a periodic cubic spline through all raw midpoints.
    Returns (smooth_cl_px, cs_x, cs_y, t_eval).
    """
    diffs   = np.diff(raw_cl, axis=0, append=raw_cl[:1])
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    t       = np.concatenate([[0.0], np.cumsum(seg_len[:-1])])
    t_total = float(np.sum(seg_len))

    # Append first point at t_total: required by CubicSpline for bc_type='periodic'
    t_full = np.append(t, t_total)
    x_full = np.append(raw_cl[:, 0], raw_cl[0, 0])
    y_full = np.append(raw_cl[:, 1], raw_cl[0, 1])

    cs_x = CubicSpline(t_full, x_full, bc_type='periodic')
    cs_y = CubicSpline(t_full, y_full, bc_type='periodic')

    t_eval    = np.linspace(0.0, t_total, n_out, endpoint=False)
    smooth_cl = np.column_stack([cs_x(t_eval), cs_y(t_eval)])
    return smooth_cl, cs_x, cs_y, t_eval


def _to_world(smooth_cl_px: np.ndarray, cs_x, cs_y, t_eval, meta: dict, H: int):
    """
    Convert pixel centerline to world frame and compute heading.

    Pixel convention: col = x (right), row = y (DOWN).
    ROS convention:   x (right), y (UP).

    World conversion:
        x_w = origin_x + col * resolution
        y_w = origin_y + (H - row) * resolution

    Yaw from spline tangent — the y-flip negates the row derivative:
        yaw = atan2(-dy_pixel, dx_pixel)
    """
    res      = meta["resolution"]
    origin_x = meta["origin"][0]
    origin_y = meta["origin"][1]

    x_world = origin_x + smooth_cl_px[:, 0] * res
    y_world = origin_y + (H - smooth_cl_px[:, 1]) * res

    dx = cs_x(t_eval, 1)   # pixel column derivative (right = positive)
    dy = cs_y(t_eval, 1)   # pixel row derivative    (down  = positive)
    yaw = np.arctan2(-dy, dx)   # negate dy to convert image → world y

    return x_world, y_world, yaw


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Extract racetrack centerline from an occupancy-grid map.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--map",     required=True,
                        help="Path to map image (.pgm or .png)")
    parser.add_argument("--yaml",    required=True,
                        help="Path to map YAML")
    parser.add_argument("--output",  default=None,
                        help="Override the output CSV filename (still placed in the run folder)")
    parser.add_argument("--spacing", type=float, default=DEFAULT_SPACING,
                        help="Target waypoint spacing (metres)")
    parser.add_argument("--plot",    action="store_true",
                        help="Show result plot")
    args = parser.parse_args()

    circuit_name = os.path.basename(args.map).split("_map")[0]
    run_dir      = os.path.join(os.getcwd(), f"{circuit_name}_{date.today().isoformat()}")
    os.makedirs(run_dir, exist_ok=True)
    shutil.copy2(args.map,  run_dir)
    shutil.copy2(args.yaml, run_dir)
    output_path  = args.output or os.path.join(run_dir, "centerline_output.csv")

    print(f"Map:     {args.map}")
    print(f"YAML:    {args.yaml}")
    print(f"Output:  {run_dir}")
    print(f"Spacing: {args.spacing} m")

    meta, img = _load(args.map, args.yaml)
    res = meta["resolution"]

    if _is_diagram_png(img):
        print("Diagram PNG detected — applying preprocessing in memory.")
        img = _preprocess_diagram(img)

    free_mask     = _build_free_mask(img)
    outer, inner  = _find_walls(free_mask)

    outer_perim_m = float(np.sum(
        np.linalg.norm(np.diff(outer, axis=0, append=outer[:1]), axis=1)
    )) * res
    n_pts = max(30, round(outer_perim_m / args.spacing))
    print(f"Outer perimeter ≈ {outer_perim_m:.1f} m  →  n_pts  = {n_pts}")

    outer_r = _resample(outer, n_pts)
    raw_cl  = _raw_midpoints(outer_r, inner)

    cl_perim_m = float(np.sum(
        np.linalg.norm(np.diff(raw_cl, axis=0, append=raw_cl[:1]), axis=1)
    )) * res
    n_out = max(30, round(cl_perim_m / args.spacing))
    print(f"Centerline arc  ≈ {cl_perim_m:.1f} m  →  n_out  = {n_out}")

    smooth_cl_px, cs_x, cs_y, t_eval = _fit_spline(raw_cl, n_out)

    sp_px = np.linalg.norm(np.diff(smooth_cl_px, axis=0, append=smooth_cl_px[:1]), axis=1)
    print(f"Output spacing  — mean: {sp_px.mean() * res:.3f} m  target: {args.spacing} m")

    x_world, y_world, yaw = _to_world(smooth_cl_px, cs_x, cs_y, t_eval, meta, img.shape[0])

    data = np.column_stack([x_world, y_world, yaw])
    np.savetxt(output_path, data, delimiter=",",
               header="x_meters,y_meters,yaw_rad", comments="")
    print(f"Saved {len(data)} waypoints → {output_path}")
    print(f"  Bounding box:  x [{x_world.min():.2f}, {x_world.max():.2f}] m"
          f"   y [{y_world.min():.2f}, {y_world.max():.2f}] m")

    if args.plot:
        import matplotlib.pyplot as plt

        outer_c    = np.vstack([outer,       outer[:1]])
        inner_c    = np.vstack([inner,       inner[:1]])
        smooth_c   = np.vstack([smooth_cl_px, smooth_cl_px[:1]])

        fig, ax = plt.subplots(figsize=(9, 7))
        ax.set_facecolor("black")
        ax.plot(outer_c[:, 0],  outer_c[:, 1],  color="red",   lw=1,   label="Outer wall")
        ax.plot(inner_c[:, 0],  inner_c[:, 1],  color="blue",  lw=1,   label="Inner wall")
        ax.plot(smooth_c[:, 0], smooth_c[:, 1], color="lime",  lw=2,
                label=f"Centerline ({n_out} pts)")
        ax.scatter(smooth_cl_px[0, 0], smooth_cl_px[0, 1],
                   color="red", s=60, zorder=5, label="Start (wp 0)")
        ax.invert_yaxis()
        ax.set_aspect("equal")
        ax.legend()
        ax.set_title(f"Centerline — {os.path.basename(args.map)}", fontsize=12, fontweight="bold")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
