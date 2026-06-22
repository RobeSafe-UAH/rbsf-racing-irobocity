"""
Racetrack centerline extraction — SOLUTION FILE.

Seven self-contained exercises that build the full algorithm step by step.
Each exercise adds one concept on top of the previous result.

Supported maps (run preprocess_maps.py once for the PNG diagrams):
    Catalunya_map_processed.png  +  Catalunya_map.yaml   (diagram PNG)
    SaoPaulo_map_processed.png   +  SaoPaulo_map.yaml
    YasMarina_map_processed.png  +  YasMarina_map.yaml
    pasillo_map.pgm              +  pasillo_map.yaml      (ROS occupancy grid)

To switch maps, edit MAP_PATH and YAML_PATH near the top of this file.

Run:
    uv run scripts/centerline/centerline_solution.py

Output (created next to this script):
    {CIRCUIT_NAME}_{YYYY-MM-DD}/
        <original map image>     — copy of the source PNG/PGM
        <original map YAML>      — copy of the source YAML
        centerline_output.csv    — x_meters, y_meters  (n_out waypoints, no duplicate)

The CSV is designed to be published as nav_msgs/Path for cyclic following
(controller iterates with  idx % n_waypoints — no duplicate first point).
"""

import os
import shutil
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
from datetime import date
from scipy.interpolate import CubicSpline

# ── Configuration ──────────────────────────────────────────────────────────────

SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
RACETRACKS_DIR = os.path.join(SCRIPT_DIR, "..", "f1tenth_racetracks")

MAP_PATH   = os.path.join(RACETRACKS_DIR, "Catalunya", "Catalunya_map_processed.png")
YAML_PATH  = os.path.join(RACETRACKS_DIR, "Catalunya", "Catalunya_map.yaml")

# MAP_PATH = os.path.join(SCRIPT_DIR, "pasillo_map.pgm")
# YAML_PATH = os.path.join(SCRIPT_DIR, "pasillo_map.yaml")

CIRCUIT_NAME    = os.path.basename(MAP_PATH).split("_map")[0]
POINT_SPACING_M = 0.50   # target arc-length spacing between output points (metres)


# ── Utility ────────────────────────────────────────────────────────────────────

def _pause(title: str) -> None:
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)
    input(f"\n[{title}] Press ENTER to continue to the next exercise…\n")
    plt.close("all")


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 1 — Load the Map and Detect Walls
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# All map images we use share the same pixel-intensity convention after
# pre-processing:
#
#   pixel > 240  →  free / drivable space  (white)
#   pixel < 80   →  wall / obstacle        (dark)
#   80 – 240     →  unknown / outside      (gray)
#
# A single threshold separates walls from everything else:
#
#   cv2.THRESH_BINARY_INV  →  pixels below the threshold become 255 (walls),
#                              pixels above become 0.
#
# The map YAML stores physical metadata (resolution in m/px, origin in
# world metres) that we will need in Exercise 7.

def ex1_load_and_threshold(map_path: str, yaml_path: str):
    """Load a map image + YAML; return the grayscale image and a binary wall mask."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)

    img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(
            f"Cannot read map image: {map_path}\n"
            "For diagram PNGs, run  python preprocess_maps.py  first."
        )

    # ── TODO 1-A ───────────────────────────────────────────────────────────────
    # Create a binary wall mask.
    # Dark pixels (intensity < 80) are walls; everything else is free or unknown.
    # Use cv2.threshold with THRESH_BINARY_INV so walls = 255, rest = 0.
    _, walls = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY_INV)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    axes[0].imshow(img, cmap="gray")
    axes[0].set_title("Original map")

    # ── TODO 1-B ───────────────────────────────────────────────────────────────
    # Show the wall mask on axes[1] and give it a title.
    axes[1].imshow(walls, cmap="gray")
    axes[1].set_title("Wall mask (Ex 1)")

    overlay = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    overlay[walls == 255] = [220, 80, 80]
    axes[2].imshow(overlay)
    axes[2].set_title("Walls highlighted")

    for ax in axes:
        ax.axis("off")
    fig.suptitle("Exercise 1 — Load & Threshold", fontsize=13, fontweight="bold")
    _pause("Ex 1")

    print(f"[Ex 1] {img.shape[1]}×{img.shape[0]} px  "
          f"resolution={meta['resolution']} m/px  "
          f"wall pixels={int(walls.sum() // 255)}")
    return meta, img, walls


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 2 — Build the Free-Space Mask
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# After preprocessing, pixels fall into three distinct intensity bands:
#
#   > 240  (white) →  drivable corridor  ← the only region we want
#   ≈ 127  (gray)  →  outer background and infield  (already excluded)
#   < 80   (dark)  →  wall lines
#
# A simple threshold at 240 isolates the corridor directly — no need to
# invert a wall mask.  This only works because preprocess_maps.py already
# separated the corridor (white) from background and infield (gray).
#
# Morphological OPENING (erosion followed by dilation) removes thin noise
# specks introduced by anti-aliasing or LiDAR quantisation while preserving
# the corridor region:
#
#   cv2.morphologyEx(src, cv2.MORPH_OPEN, kernel)
#
# Result: a binary mask where 255 = drivable corridor ring only.

def ex2_build_free_mask(img: np.ndarray) -> np.ndarray:
    """Threshold for white corridor pixels and clean noise with morphological opening."""
    _, free_raw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    kernel      = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    # ── TODO 2-A ───────────────────────────────────────────────────────────────
    # Remove isolated noise pixels from free_raw using morphological OPEN.
    # Assign the cleaned result to free_mask.
    free_mask = cv2.morphologyEx(free_raw, cv2.MORPH_OPEN, kernel)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(free_raw,  cmap="gray")
    axes[0].set_title("Free space — raw (before opening)")
    axes[1].imshow(free_mask, cmap="gray")
    axes[1].set_title("Free-space mask (Ex 2)")
    for ax in axes:
        ax.axis("off")
    fig.suptitle("Exercise 2 — Free-Space Mask", fontsize=13, fontweight="bold")
    _pause("Ex 2")

    print(f"[Ex 2] Free pixels: {int(free_mask.sum() // 255)}")
    return free_mask


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 3 — Extract Inner and Outer Wall Contours
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# cv2.findContours with RETR_CCOMP returns every contour together with a
# two-level parent-child hierarchy:
#
#   hierarchy[i][3] == -1   →  level 0: outer boundary of a white blob
#   hierarchy[i][3] != -1   →  level 1: hole inside a white blob
#
# For a ring-shaped free-space mask (corridor only, after preprocessing):
#   • One large OUTER contour  — the track's outer edge
#   • One large HOLE contour   — the dark hole in the ring  =  the inner wall
#
# Selecting the largest of each by area is sufficient for our maps.
# The production script adds a proximity filter to reject infield regions or
# map artifacts on noisier maps.

def ex3_find_wall_contours(free_mask: np.ndarray):
    """Return (outer, inner) wall contours as float32 (N, 2) pixel arrays."""
    contours, hierarchy = cv2.findContours(
        free_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    if hierarchy is None or len(contours) < 2:
        raise RuntimeError("Expected at least 2 contours. Check the free-space mask.")
    hierarchy = hierarchy[0]

    outer_contours = [c.reshape(-1, 2) for i, c in enumerate(contours)
                      if hierarchy[i][3] == -1]
    hole_contours  = [c.reshape(-1, 2) for i, c in enumerate(contours)
                      if hierarchy[i][3] != -1]

    # ── TODO 3-A ───────────────────────────────────────────────────────────────
    # Select the largest outer contour by enclosed area.
    # Hint: max(outer_contours, key=lambda c: cv2.contourArea(c))
    # Cast the result to float32.
    outer = max(outer_contours, key=lambda c: cv2.contourArea(c)).astype(np.float32)

    # ── TODO 3-B ───────────────────────────────────────────────────────────────
    # Select the largest hole contour (= the inner wall).
    # Cast to float32.
    inner = max(hole_contours,  key=lambda c: cv2.contourArea(c)).astype(np.float32)

    outer_c = np.vstack([outer, outer[:1]])
    inner_c = np.vstack([inner, inner[:1]])
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_facecolor("black")
    ax.plot(outer_c[:, 0], outer_c[:, 1], color="red",  linewidth=1.5, label="Outer wall")
    ax.plot(inner_c[:, 0], inner_c[:, 1], color="blue", linewidth=1.5, label="Inner wall")
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.legend()
    ax.set_title("Exercise 3 — Wall Contours", fontsize=13, fontweight="bold")
    _pause("Ex 3")

    print(f"[Ex 3] Outer: {len(outer)} pts  |  Inner: {len(inner)} pts")
    return outer, inner


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 4 — Arc-Length Resampling
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# Raw contours from findContours have highly non-uniform spacing: many points
# on curves, very few on straight sections.
#
# Arc-length resampling gives exactly N evenly-spaced points:
#
#   1. Compute the length of each segment between consecutive points.
#   2. Build the cumulative arc-length array s: s[0]=0, s[k]=sum of first k lengths.
#   3. Choose N target positions t uniformly in [0, total_length).
#   4. Linearly interpolate x and y coordinates at those positions.
#
# np.interp handles step 4.  We extend the arrays by one point (closing the
# loop) before interpolating so the wrap-around segment is covered correctly.

def ex4_resample_arc_length(pts: np.ndarray, n: int) -> np.ndarray:
    """Resample a closed contour to n evenly-spaced points by arc length."""
    pts     = pts.astype(np.float64)
    diffs   = np.diff(pts, axis=0, append=pts[:1])       # wrap-around differences
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])         # length of each segment

    # ── TODO 4-A ───────────────────────────────────────────────────────────────
    # Compute the cumulative arc length.
    # cum_len[0] = 0.0; cum_len[k] = sum of the first k segment lengths.
    # Hint: np.concatenate([[0.0], np.cumsum(seg_len)])
    cum_len = np.concatenate([[0.0], np.cumsum(seg_len)])
    total   = cum_len[-1]

    # Close the coordinate arrays so np.interp can handle the last→first segment.
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])

    t = np.linspace(0.0, total, n, endpoint=False)       # N equally-spaced targets

    # ── TODO 4-B ───────────────────────────────────────────────────────────────
    # Interpolate x and y at each target position t.
    # Hint: np.interp(t, cum_len, x_ext)
    x_new = np.interp(t, cum_len, x_ext)
    y_new = np.interp(t, cum_len, y_ext)

    return np.column_stack([x_new, y_new])


def _show_ex4(outer: np.ndarray, outer_r: np.ndarray) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for ax in axes:
        ax.set_facecolor("black")
        ax.set_aspect("equal")
        ax.invert_yaxis()

    axes[0].scatter(outer[:, 0],   outer[:, 1],   s=1,  color="gray",   label=f"Raw ({len(outer)} pts)")
    axes[0].set_title(f"Raw outer contour — uneven spacing")
    axes[0].legend(markerscale=6)

    axes[1].scatter(outer_r[:, 0], outer_r[:, 1], s=8,  color="orange", label=f"Resampled ({len(outer_r)} pts)")
    axes[1].set_title(f"After arc-length resampling — uniform spacing")
    axes[1].legend(markerscale=3)

    fig.suptitle("Exercise 4 — Arc-Length Resampling", fontsize=13, fontweight="bold")
    _pause("Ex 4")

    sp = np.linalg.norm(np.diff(outer_r, axis=0, append=outer_r[:1]), axis=1)
    print(f"[Ex 4] Spacing (px) — min: {sp.min():.2f}  mean: {sp.mean():.2f}  max: {sp.max():.2f}")


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 5 — Raw Centerline by Midpoint Matching
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# With the outer wall sampled at N uniformly-spaced points we compute the
# centerline by pairing each outer point with its nearest inner-wall point
# and taking the midpoint:
#
#   j* = argmin_j  ||outer[i] − inner[j]||²
#   centerline[i] = (outer[i] + inner[j*]) / 2
#
# NumPy broadcasting computes all N × M squared distances at once:
#
#   diffs  shape: (N, 1, 2) − (1, M, 2)  →  (N, M, 2)
#   sq_dist       (diffs**2).sum(axis=2)  →  (N, M)
#   idx           argmin along axis=1     →  (N,)

def ex5_raw_centerline(outer_r: np.ndarray, inner: np.ndarray) -> np.ndarray:
    """
    Match each outer-wall point to its nearest inner-wall point.
    Return the midpoints as the raw (unsmoothed) centerline.
    """
    inner_f = inner.astype(np.float64)
    diffs   = outer_r[:, None, :] - inner_f[None, :, :]   # (N, M, 2)

    # ── TODO 5-A ───────────────────────────────────────────────────────────────
    # For each outer point find the index of the nearest inner-wall point.
    # Use argmin on (diffs**2).sum(axis=2) along axis=1.
    idx = np.argmin((diffs ** 2).sum(axis=2), axis=1)     # (N,)

    inner_matched = inner_f[idx]                           # (N, 2)

    # ── TODO 5-B ───────────────────────────────────────────────────────────────
    # Compute the raw centerline as the midpoint of each (outer, inner) pair.
    raw_cl = (outer_r + inner_matched) / 2.0

    widths_px = np.linalg.norm(outer_r - inner_matched, axis=1)
    print(f"[Ex 5] Track width (px) — min: {widths_px.min():.1f}  "
          f"mean: {widths_px.mean():.1f}  max: {widths_px.max():.1f}")

    outer_rc = np.vstack([outer_r, outer_r[:1]])
    inner_fc = np.vstack([inner_f, inner_f[:1]])
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_facecolor("black")
    ax.plot(outer_rc[:, 0], outer_rc[:, 1], color="red",    linewidth=1,   label="Outer wall")
    ax.plot(inner_fc[:, 0], inner_fc[:, 1], color="blue",   linewidth=1,   label="Inner wall")
    ax.scatter(raw_cl[:, 0], raw_cl[:, 1],  color="yellow", s=4,           label="Raw centerline")
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.legend(markerscale=4)
    ax.set_title("Exercise 5 — Raw Centerline", fontsize=13, fontweight="bold")
    _pause("Ex 5")

    return raw_cl


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 6 — Periodic Cubic Spline Interpolation
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# Savitzky-Golay smoothing APPROXIMATES the data: each output point is a
# weighted average of its neighbours, so the curve passes NEAR the raw
# midpoints but not through them (it shortens corners).
#
# A periodic cubic spline INTERPOLATES the data: the curve passes EXACTLY
# through every input point and is C² smooth (continuous curvature) between
# them.  This is the right choice when the midpoints are already good
# estimates of where the centreline should be.
#
# scipy.interpolate.CubicSpline(t, y, bc_type='periodic') fits piecewise
# cubics through all N control points with:
#   • exact interpolation at every point
#   • continuous first and second derivatives everywhere
#   • seamless join at the loop boundary  (requires y[0] == y[-1])
#
# We parameterise by cumulative arc length t, append the first point at
# t = t_total to close the loop, then evaluate at N_POINTS_OUT uniform
# t values to get a dense, smooth output.

def ex6_smooth_centerline(raw_cl: np.ndarray, n_out: int) -> np.ndarray:
    """Fit a periodic cubic spline through all raw midpoints."""
    # Arc-length parameterisation
    diffs   = np.diff(raw_cl, axis=0, append=raw_cl[:1])
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    t       = np.concatenate([[0.0], np.cumsum(seg_len[:-1])])  # N values
    t_total = float(np.sum(seg_len))

    # Periodic BC requires y[0] == y[-1]: append first point at t = t_total
    t_full = np.append(t, t_total)
    x_full = np.append(raw_cl[:, 0], raw_cl[0, 0])
    y_full = np.append(raw_cl[:, 1], raw_cl[0, 1])

    # ── TODO 6-A ───────────────────────────────────────────────────────────────
    # Fit periodic cubic splines for x and y.
    # Use CubicSpline(t_full, x_full, bc_type='periodic') and the same for y.
    # bc_type='periodic' ensures the curve joins smoothly at the loop seam.
    cs_x = CubicSpline(t_full, x_full, bc_type='periodic')
    cs_y = CubicSpline(t_full, y_full, bc_type='periodic')

    # Evaluate at n_out uniform arc-length positions
    t_eval    = np.linspace(0.0, t_total, n_out, endpoint=False)
    smooth_cl = np.column_stack([cs_x(t_eval), cs_y(t_eval)])

    smooth_c = np.vstack([smooth_cl, smooth_cl[:1]])
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_facecolor("black")
    ax.scatter(raw_cl[:, 0],   raw_cl[:, 1],   color="yellow", s=4,        label=f"Raw midpoints ({len(raw_cl)} pts)")
    ax.plot(smooth_c[:, 0],    smooth_c[:, 1],  color="lime",  linewidth=2,
            label=f"Cubic spline ({n_out} pts)")
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.legend(markerscale=4)
    ax.set_title("Exercise 6 — Cubic Spline Interpolation", fontsize=13, fontweight="bold")
    _pause("Ex 6")

    return smooth_cl


# ══════════════════════════════════════════════════════════════════════════════
# EXERCISE 7 — World-Frame Coordinate Export
# ══════════════════════════════════════════════════════════════════════════════
#
# CONCEPT
# -------
# Everything so far is in pixel coordinates: column = x, row = y (y grows DOWN).
# ROS uses a different convention: y grows UPWARD from the map's origin.
#
# The YAML 'origin' field is the world position of the map's bottom-left corner.
# Conversion from pixel (col, row) to world (x_w, y_w):
#
#   x_w = origin_x + col * resolution
#   y_w = origin_y + (H − row) * resolution
#
# The (H − row) term flips the y-axis: row 0 is the top of the image, but
# the bottom-left corner is the map origin, so row 0 maps to y = H * resolution.
#
# The exported CSV is the format consumed by the ROS centerline publisher node.

def ex7_world_frame_export(smooth_cl: np.ndarray, meta: dict, img: np.ndarray,
                           output_csv: str) -> None:
    """Convert the centerline from pixel coords to world metres and save CSV."""
    res      = meta["resolution"]
    origin_x = meta["origin"][0]
    origin_y = meta["origin"][1]
    H        = img.shape[0]            # image height in pixels

    # ── TODO 7-A ───────────────────────────────────────────────────────────────
    # Convert pixel column (smooth_cl[:, 0]) to world x.
    # x_world = origin_x + col * resolution
    x_world = origin_x + smooth_cl[:, 0] * res

    # ── TODO 7-B ───────────────────────────────────────────────────────────────
    # Convert pixel row (smooth_cl[:, 1]) to world y.
    # Remember: ROS y grows UP, image rows grow DOWN.
    # y_world = origin_y + (H - row) * resolution
    y_world = origin_y + (H - smooth_cl[:, 1]) * res

    data = np.column_stack([x_world, y_world])
    np.savetxt(output_csv, data, delimiter=",",
               header="x_meters,y_meters", comments="")
    print(f"[Ex 7] Saved {len(data)} points to: {output_csv}")
    print(f"[Ex 7] World bounding box:")
    print(f"         x: [{x_world.min():.2f}, {x_world.max():.2f}] m")
    print(f"         y: [{y_world.min():.2f}, {y_world.max():.2f}] m")

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.plot(np.append(x_world, x_world[0]), np.append(y_world, y_world[0]), color="lime", linewidth=2)
    ax.scatter(x_world[0], y_world[0], color="red", s=80, zorder=5, label="Start")
    ax.set_xlabel("x  [m]")
    ax.set_ylabel("y  [m]")
    ax.set_aspect("equal")
    ax.legend()
    ax.set_title("Exercise 7 — Centerline in World Frame", fontsize=13, fontweight="bold")
    _pause("Ex 7")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    run_dir = os.path.join(SCRIPT_DIR, f"{CIRCUIT_NAME}_{date.today().isoformat()}")
    os.makedirs(run_dir, exist_ok=True)
    shutil.copy2(MAP_PATH,  run_dir)
    shutil.copy2(YAML_PATH, run_dir)
    output_csv = os.path.join(run_dir, "centerline_output.csv")

    print("=" * 60)
    print("  Racetrack Centerline Extraction  —  SOLUTION")
    print("=" * 60)
    print(f"  Map:    {MAP_PATH}")
    print(f"  YAML:   {YAML_PATH}")
    print(f"  Output: {run_dir}\n")

    meta, img, walls = ex1_load_and_threshold(MAP_PATH, YAML_PATH)
    free_mask        = ex2_build_free_mask(img)
    outer, inner     = ex3_find_wall_contours(free_mask)

    res = meta['resolution']
    outer_perim_m = float(np.sum(np.linalg.norm(
        np.diff(outer, axis=0, append=outer[:1]), axis=1))) * res
    n_pts = max(30, round(outer_perim_m / POINT_SPACING_M))
    print(f"[main] Outer perimeter ≈ {outer_perim_m:.1f} m  →  n_points = {n_pts}")

    outer_r          = ex4_resample_arc_length(outer, n_pts)
    _show_ex4(outer, outer_r)
    raw_cl           = ex5_raw_centerline(outer_r, inner)

    cl_perim_m = float(np.sum(np.linalg.norm(
        np.diff(raw_cl, axis=0, append=raw_cl[:1]), axis=1))) * res
    n_out = max(30, round(cl_perim_m / POINT_SPACING_M))
    print(f"[main] Centerline arc  ≈ {cl_perim_m:.1f} m  →  n_out      = {n_out}")

    smooth_cl        = ex6_smooth_centerline(raw_cl, n_out)
    sp_px = np.linalg.norm(np.diff(smooth_cl, axis=0, append=smooth_cl[:1]), axis=1)
    print(f"[main] Output spacing   — mean: {sp_px.mean() * res:.3f} m  target: {POINT_SPACING_M} m")
    ex7_world_frame_export(smooth_cl, meta, img, output_csv)

    print("\nAll exercises complete.")


if __name__ == "__main__":
    main()
