"""
Racetrack centerline extraction -- SOLUTION FILE (headless / batch mode).

Identical to centerline_solution.py but saves every intermediate figure to
the output directory instead of pausing for interactive display.  Suitable
for running without a monitor or inside a CI pipeline.

Output (created next to this script):
    {CIRCUIT_NAME}_{YYYY-MM-DD}/
        <original map image>
        <original map YAML>
        centerline_output.csv    -- x_meters, y_meters  (n_out waypoints)
        ex1_load_threshold.png
        ex2_free_mask.png
        ex3_wall_contours.png
        ex4_arc_length_resampling.png
        ex5_raw_centerline.png
        ex6_cubic_spline.png
        ex7_world_frame.png

Run:
    uv run scripts/centerline/centerline_solution_headless.py
"""

import os
import shutil
import yaml
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")   # non-interactive backend -- no display required
import matplotlib.pyplot as plt
from datetime import date
from scipy.interpolate import CubicSpline

# -- Configuration --------------------------------------------------------------

SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
RACETRACKS_DIR = os.path.join(SCRIPT_DIR, "..", "f1tenth_racetracks")

MAP_PATH   = os.path.join(RACETRACKS_DIR, "Catalunya", "Catalunya_map_processed.png")
YAML_PATH  = os.path.join(RACETRACKS_DIR, "Catalunya", "Catalunya_map.yaml")

# MAP_PATH = os.path.join(SCRIPT_DIR, "pasillo_map.pgm")
# YAML_PATH = os.path.join(SCRIPT_DIR, "pasillo_map.yaml")

CIRCUIT_NAME    = os.path.basename(MAP_PATH).split("_map")[0]
POINT_SPACING_M = 0.50   # target arc-length spacing between output points (metres)

_OUT_DIR: str = ""       # set by main() before any exercise runs


# -- Utility --------------------------------------------------------------------

def _save(title: str, filename: str) -> None:
    plt.tight_layout()
    out_path = os.path.join(_OUT_DIR, filename)
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close("all")
    print(f"  [{title}] saved -> {out_path}")


# ==============================================================================
# EXERCISE 1 -- Load the Map and Detect Walls
# ==============================================================================

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

    _, walls = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY_INV)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    axes[0].imshow(img, cmap="gray")
    axes[0].set_title("Original map")
    axes[1].imshow(walls, cmap="gray")
    axes[1].set_title("Wall mask (Ex 1)")
    overlay = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    overlay[walls == 255] = [220, 80, 80]
    axes[2].imshow(overlay)
    axes[2].set_title("Walls highlighted")
    for ax in axes:
        ax.axis("off")
    fig.suptitle("Exercise 1 -- Load & Threshold", fontsize=13, fontweight="bold")
    _save("Ex 1", "ex1_load_threshold.png")

    print(f"[Ex 1] {img.shape[1]}x{img.shape[0]} px  "
          f"resolution={meta['resolution']} m/px  "
          f"wall pixels={int(walls.sum() // 255)}")
    return meta, img, walls


# ==============================================================================
# EXERCISE 2 -- Build the Free-Space Mask
# ==============================================================================

def ex2_build_free_mask(img: np.ndarray) -> np.ndarray:
    """Threshold for white corridor pixels and clean noise with morphological opening."""
    _, free_raw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    kernel      = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    free_mask   = cv2.morphologyEx(free_raw, cv2.MORPH_OPEN, kernel)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].imshow(free_raw,  cmap="gray")
    axes[0].set_title("Free space -- raw (before opening)")
    axes[1].imshow(free_mask, cmap="gray")
    axes[1].set_title("Free-space mask (Ex 2)")
    for ax in axes:
        ax.axis("off")
    fig.suptitle("Exercise 2 -- Free-Space Mask", fontsize=13, fontweight="bold")
    _save("Ex 2", "ex2_free_mask.png")

    print(f"[Ex 2] Free pixels: {int(free_mask.sum() // 255)}")
    return free_mask


# ==============================================================================
# EXERCISE 3 -- Extract Inner and Outer Wall Contours
# ==============================================================================

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

    outer = max(outer_contours, key=lambda c: cv2.contourArea(c)).astype(np.float32)
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
    ax.set_title("Exercise 3 -- Wall Contours", fontsize=13, fontweight="bold")
    _save("Ex 3", "ex3_wall_contours.png")

    print(f"[Ex 3] Outer: {len(outer)} pts  |  Inner: {len(inner)} pts")
    return outer, inner


# ==============================================================================
# EXERCISE 4 -- Arc-Length Resampling
# ==============================================================================

def ex4_resample_arc_length(pts: np.ndarray, n: int) -> np.ndarray:
    """Resample a closed contour to n evenly-spaced points by arc length."""
    pts     = pts.astype(np.float64)
    diffs   = np.diff(pts, axis=0, append=pts[:1])
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    cum_len = np.concatenate([[0.0], np.cumsum(seg_len)])
    total   = cum_len[-1]

    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])
    t     = np.linspace(0.0, total, n, endpoint=False)

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
    axes[0].set_title("Raw outer contour -- uneven spacing")
    axes[0].legend(markerscale=6)

    axes[1].scatter(outer_r[:, 0], outer_r[:, 1], s=8,  color="orange", label=f"Resampled ({len(outer_r)} pts)")
    axes[1].set_title("After arc-length resampling -- uniform spacing")
    axes[1].legend(markerscale=3)

    fig.suptitle("Exercise 4 -- Arc-Length Resampling", fontsize=13, fontweight="bold")
    _save("Ex 4", "ex4_arc_length_resampling.png")

    sp = np.linalg.norm(np.diff(outer_r, axis=0, append=outer_r[:1]), axis=1)
    print(f"[Ex 4] Spacing (px) -- min: {sp.min():.2f}  mean: {sp.mean():.2f}  max: {sp.max():.2f}")


# ==============================================================================
# EXERCISE 5 -- Raw Centerline by Midpoint Matching
# ==============================================================================

def ex5_raw_centerline(outer_r: np.ndarray, inner: np.ndarray) -> np.ndarray:
    """
    Match each outer-wall point to its nearest inner-wall point.
    Return the midpoints as the raw (unsmoothed) centerline.
    """
    inner_f = inner.astype(np.float64)
    diffs   = outer_r[:, None, :] - inner_f[None, :, :]
    idx     = np.argmin((diffs ** 2).sum(axis=2), axis=1)

    inner_matched = inner_f[idx]
    raw_cl        = (outer_r + inner_matched) / 2.0

    widths_px = np.linalg.norm(outer_r - inner_matched, axis=1)
    print(f"[Ex 5] Track width (px) -- min: {widths_px.min():.1f}  "
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
    ax.set_title("Exercise 5 -- Raw Centerline", fontsize=13, fontweight="bold")
    _save("Ex 5", "ex5_raw_centerline.png")

    return raw_cl


# ==============================================================================
# EXERCISE 6 -- Periodic Cubic Spline Interpolation
# ==============================================================================

def ex6_smooth_centerline(raw_cl: np.ndarray, n_out: int) -> np.ndarray:
    """Fit a periodic cubic spline through all raw midpoints."""
    diffs   = np.diff(raw_cl, axis=0, append=raw_cl[:1])
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    t       = np.concatenate([[0.0], np.cumsum(seg_len[:-1])])
    t_total = float(np.sum(seg_len))

    t_full = np.append(t, t_total)
    x_full = np.append(raw_cl[:, 0], raw_cl[0, 0])
    y_full = np.append(raw_cl[:, 1], raw_cl[0, 1])

    cs_x = CubicSpline(t_full, x_full, bc_type='periodic')
    cs_y = CubicSpline(t_full, y_full, bc_type='periodic')

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
    ax.set_title("Exercise 6 -- Cubic Spline Interpolation", fontsize=13, fontweight="bold")
    _save("Ex 6", "ex6_cubic_spline.png")

    return smooth_cl


# ==============================================================================
# EXERCISE 7 -- World-Frame Coordinate Export
# ==============================================================================

def ex7_world_frame_export(smooth_cl: np.ndarray, meta: dict, img: np.ndarray,
                           output_csv: str) -> None:
    """Convert the centerline from pixel coords to world metres and save CSV."""
    res      = meta["resolution"]
    origin_x = meta["origin"][0]
    origin_y = meta["origin"][1]
    H        = img.shape[0]

    x_world = origin_x + smooth_cl[:, 0] * res
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
    ax.set_title("Exercise 7 -- Centerline in World Frame", fontsize=13, fontweight="bold")
    _save("Ex 7", "ex7_world_frame.png")


# -- Main -----------------------------------------------------------------------

def main():
    global _OUT_DIR
    _OUT_DIR = os.path.join(SCRIPT_DIR, f"{CIRCUIT_NAME}_{date.today().isoformat()}")
    os.makedirs(_OUT_DIR, exist_ok=True)
    shutil.copy2(MAP_PATH,  _OUT_DIR)
    shutil.copy2(YAML_PATH, _OUT_DIR)
    output_csv = os.path.join(_OUT_DIR, "centerline_output.csv")

    print("=" * 60)
    print("  Racetrack Centerline Extraction  --  SOLUTION (headless)")
    print("=" * 60)
    print(f"  Map:    {MAP_PATH}")
    print(f"  YAML:   {YAML_PATH}")
    print(f"  Output: {_OUT_DIR}\n")

    meta, img, walls = ex1_load_and_threshold(MAP_PATH, YAML_PATH)
    free_mask        = ex2_build_free_mask(img)
    outer, inner     = ex3_find_wall_contours(free_mask)

    res = meta['resolution']
    outer_perim_m = float(np.sum(np.linalg.norm(
        np.diff(outer, axis=0, append=outer[:1]), axis=1))) * res
    n_pts = max(30, round(outer_perim_m / POINT_SPACING_M))
    print(f"[main] Outer perimeter ~ {outer_perim_m:.1f} m  ->  n_points = {n_pts}")

    outer_r          = ex4_resample_arc_length(outer, n_pts)
    _show_ex4(outer, outer_r)
    raw_cl           = ex5_raw_centerline(outer_r, inner)

    cl_perim_m = float(np.sum(np.linalg.norm(
        np.diff(raw_cl, axis=0, append=raw_cl[:1]), axis=1))) * res
    n_out = max(30, round(cl_perim_m / POINT_SPACING_M))
    print(f"[main] Centerline arc  ~ {cl_perim_m:.1f} m  ->  n_out      = {n_out}")

    smooth_cl        = ex6_smooth_centerline(raw_cl, n_out)
    sp_px = np.linalg.norm(np.diff(smooth_cl, axis=0, append=smooth_cl[:1]), axis=1)
    print(f"[main] Output spacing   -- mean: {sp_px.mean() * res:.3f} m  target: {POINT_SPACING_M} m")
    ex7_world_frame_export(smooth_cl, meta, img, output_csv)

    print("\nAll exercises complete.")


if __name__ == "__main__":
    main()
