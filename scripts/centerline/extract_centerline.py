"""
Racetrack centerline extraction from ROS occupancy-grid or diagram-style PNG maps.

Public API
----------
extract_centerline_and_distances(yaml_path, map_path, ...) -> tuple
    Load a map, extract the geometric centerline of the track corridor,
    smooth it, and return it alongside a distance-transform map.
"""

import argparse
import os
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt


# ── Map loading & free-space mask ─────────────────────────────────────────────

def load_ros_map(yaml_path, map_path):
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    grid_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if grid_img is None:
        raise ValueError(
            f"Could not read map image: {map_path}. "
            "Supported formats: .pgm, .png, and any OpenCV-supported format."
        )
    return grid_img, map_metadata


def _is_diagram_map(grid_img: np.ndarray) -> bool:
    """
    Detect diagram-style maps: thin dark lines on a bright background
    (e.g. a track outline PNG), as opposed to ROS occupancy grids where
    the majority of pixels are dark (walls/unknown).
    """
    return float(np.mean(grid_img > 240)) > 0.85


def _build_free_mask_ros(grid_img: np.ndarray) -> np.ndarray:
    """
    ROS occupancy-grid convention: white (>240) = free, dark = wall.
    Morphological opening removes isolated LiDAR noise pixels.
    """
    _, binary = cv2.threshold(grid_img, 240, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)


def _build_free_mask_diagram(grid_img: np.ndarray) -> np.ndarray:
    """
    Diagram-style map: thin dark lines = walls, white = free space.

    Steps:
      1. Threshold dark pixels as walls.
      2. Close wall lines to seal any small gaps.
      3. Invert to get the raw free mask.
      4. Remove background — any connected component that touches the image
         border is outside the track and is discarded.

    What survives: only the enclosed regions (track corridor + infield).
    """
    # 1. Detect wall pixels (dark lines)
    _, walls = cv2.threshold(grid_img, 80, 255, cv2.THRESH_BINARY_INV)

    # 2. Close small gaps so wall lines form complete closed boundaries
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    walls_closed = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, close_kernel)

    # 3. Free space = everything that is not a wall
    free_raw = cv2.bitwise_not(walls_closed)

    # 4. Remove background: label all connected components and discard any
    #    that touch the image border (those are outside the track)
    _, labels = cv2.connectedComponents(free_raw)
    h, w = free_raw.shape
    border_labels = (set(labels[0,  :].tolist()) |
                     set(labels[-1, :].tolist()) |
                     set(labels[:,  0].tolist()) |
                     set(labels[:, -1].tolist()))
    free_mask = np.zeros_like(free_raw)
    for lbl in range(1, int(labels.max()) + 1):
        if lbl not in border_labels:
            free_mask[labels == lbl] = 255
    return free_mask


def build_free_mask(grid_img: np.ndarray) -> np.ndarray:
    """Dispatch to the correct preprocessing based on map style."""
    if _is_diagram_map(grid_img):
        print("[centerline] Detected diagram-style map (thin lines on white background).")
        return _build_free_mask_diagram(grid_img)
    print("[centerline] Detected ROS occupancy-grid map.")
    return _build_free_mask_ros(grid_img)


# ── Geometry helpers ───────────────────────────────────────────────────────────

def _savgol_coeffs(window_len: int, polyorder: int) -> np.ndarray:
    """
    Savitzky-Golay smoothing coefficients via polynomial least-squares.

    Fits a degree-polyorder polynomial to each window of length window_len and
    evaluates it at the centre point. Equivalent to scipy.signal.savgol_coeffs
    with deriv=0.
    """
    half = window_len // 2
    x = np.arange(-half, half + 1, dtype=np.float64)
    A = np.vander(x, polyorder + 1, increasing=True)
    return np.linalg.pinv(A)[0]   # row 0 → smoothing (deriv=0) coefficients


def _resample_contour_arc_length(pts: np.ndarray, n: int) -> np.ndarray:
    """Resample a closed contour to n evenly-spaced points by arc length."""
    pts = pts.astype(np.float64)
    diffs = np.diff(pts, axis=0, append=pts[:1])
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    cum_len = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum_len[-1]
    x_ext = np.append(pts[:, 0], pts[0, 0])
    y_ext = np.append(pts[:, 1], pts[0, 1])
    t = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(t, cum_len, x_ext),
                            np.interp(t, cum_len, y_ext)])


def _auto_window_len(raw_pts: np.ndarray, smoothing_px: float = 5.0, polyorder: int = 3) -> int:
    """
    Choose a Savitzky-Golay window that spans roughly smoothing_px of arc length.

    For dense point clouds (small maps like pasillo, spacing ≈ 0.2 px/pt) this
    yields a large window (~25) that irons out pixel-grid noise.  For sparse
    clouds (large diagram maps, spacing ≈ 7-9 px/pt) it yields a small window
    (~5) that keeps the path close to the geometric midpoint while still
    removing any residual jitter.
    """
    diffs = np.diff(raw_pts, axis=0, append=raw_pts[:1])
    perimeter_px = float(np.sum(np.linalg.norm(diffs, axis=1)))
    spacing_px = perimeter_px / len(raw_pts)
    wl = round(smoothing_px / spacing_px)
    if wl % 2 == 0:
        wl += 1
    min_w = polyorder + 1
    if min_w % 2 == 0:
        min_w += 1
    wl = max(wl, min_w)
    n = len(raw_pts)
    max_w = n if n % 2 != 0 else n - 1
    wl = min(wl, max_w)
    print(f"[centerline] Auto window_len={wl} "
          f"(perimeter={perimeter_px:.0f} px, spacing={spacing_px:.2f} px/pt)")
    return wl


# ── Centerline extraction & smoothing ─────────────────────────────────────────

def extract_raw_centerline(
    free_mask: np.ndarray,
    resolution: float,
    n_points: int = None,
    point_spacing_m: float = 0.15,
    max_track_width_m: float = 3.0,
) -> np.ndarray:
    """
    Extract the raw centerline of a ring-shaped racetrack as the midpoint
    between the inner and outer wall boundaries.

    Steps:
      1. findContours with RETR_CCOMP to get outer boundary (level 0) and
         inner hole boundary (level 1) of the free-space corridor.
      2. Among all hole contours, keep only those whose boundary points are
         within max_track_width_m of the outer wall — this rejects infield
         regions or map artifacts that are too far away to be the inner wall
         of a 1/10-scale racetrack.
      3. Resample the outer boundary to n_points evenly by arc length.
         If n_points is None, it is computed from point_spacing_m so that
         point spacing >> pixel quantization noise regardless of map scale.
      4. For each outer point find the nearest inner-wall point.
      5. Centerline = midpoint of each (outer, inner) pair.

    Returns (n_points, 2) array of (x, y) pixel coords ordered around the track.
    """
    contours, hierarchy = cv2.findContours(
        free_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    if hierarchy is None or len(contours) < 2:
        raise RuntimeError(
            "Expected at least 2 contours (outer wall and inner wall). "
            "Check that the map binarization is correct."
        )
    hierarchy = hierarchy[0]

    outer_contours = [contours[i].reshape(-1, 2)
                      for i in range(len(contours)) if hierarchy[i][3] == -1]
    hole_contours  = [contours[i].reshape(-1, 2)
                      for i in range(len(contours)) if hierarchy[i][3] != -1]

    if not outer_contours or not hole_contours:
        raise RuntimeError(
            "Could not distinguish outer and inner wall contours. "
            "The map may not represent a closed racetrack corridor."
        )

    outer = max(outer_contours, key=lambda c: cv2.contourArea(c)).astype(np.float32)

    # Keep only hole contours whose boundary sits within max_track_width_m of
    # the outer wall. For a 1/10-scale racetrack the corridor is narrow; any
    # hole that is farther away is an infield or unrelated region.
    max_track_width_px = max_track_width_m / resolution
    outer_f = outer.astype(np.float64)
    valid_holes = []
    for hole in hole_contours:
        hole_f = hole.astype(np.float64)
        # min distance from each hole point to any outer-wall point
        min_dists = np.array([
            np.min(np.linalg.norm(outer_f - pt, axis=1)) for pt in hole_f
        ])
        if min_dists.mean() <= max_track_width_px:
            valid_holes.append(hole)

    if not valid_holes:
        raise RuntimeError(
            f"No inner wall found within {max_track_width_m} m of the outer wall. "
            "The map may not be a closed racetrack, or try --max-track-width."
        )

    inner = max(valid_holes, key=lambda c: cv2.contourArea(c)).astype(np.float32)

    if n_points is None:
        outer_diffs = np.diff(outer, axis=0, append=outer[:1])
        perimeter_m = float(np.sum(np.linalg.norm(outer_diffs, axis=1))) * resolution
        n_points = max(30, min(3000, round(perimeter_m / point_spacing_m)))
        print(f"[centerline] Adaptive n_points={n_points} "
              f"(perimeter={perimeter_m:.1f} m, target_spacing={point_spacing_m} m/pt)")

    outer_resampled = _resample_contour_arc_length(outer, n_points)

    # Match each outer point to its nearest inner-wall point.
    # Chunked to keep peak memory bounded (~16 MB per chunk).
    inner_f = inner.astype(np.float64)
    idx = np.empty(len(outer_resampled), dtype=int)
    chunk = 200
    for start in range(0, len(outer_resampled), chunk):
        end = min(start + chunk, len(outer_resampled))
        diffs = outer_resampled[start:end, None, :] - inner_f[None, :, :]
        idx[start:end] = np.argmin((diffs ** 2).sum(axis=2), axis=1)
    inner_matched = inner[idx]

    # Report measured track width for diagnostics
    widths_m = np.linalg.norm(outer_resampled - inner_matched, axis=1) * resolution
    print(f"[centerline] Track width — "
          f"min: {widths_m.min():.2f} m  "
          f"mean: {widths_m.mean():.2f} m  "
          f"max: {widths_m.max():.2f} m")

    return (outer_resampled + inner_matched) / 2.0


def smooth_centerline_closed(
    raw_pts: np.ndarray,
    constraint_mask: np.ndarray,
    window_len: int = None,
    polyorder: int = 3,
) -> np.ndarray:
    """
    Smooth a closed-loop centerline with three guarantees:

    1. Closed — data is periodically wrapped before Savitzky-Golay so both
       ends of the sequence are smoothed with awareness of the other end.
       The result connects seamlessly with no jump at the seam.

    2. In-bounds — projecting to constraint_mask (the wall-inflated tight
       corridor) is done in two rounds:
         a. Project after the first SG pass.
         b. Re-smooth, then project again.
       Round (a) removes the initial overshoots; round (b) removes kinks
       that re-smoothing introduced at the newly snapped points.
       Because constraint_mask is already eroded by wall_margin, any point
       inside it is guaranteed to be inside the real walls.

    3. Smooth — kinks from projection are minimised by the two-pass design;
       the final path has no abrupt direction changes.
    """
    n = len(raw_pts)

    if window_len is None:
        window_len = _auto_window_len(raw_pts, polyorder=polyorder)

    # Clamp window_len: must be odd, > polyorder, and ≤ n
    window_len = min(window_len, n if n % 2 != 0 else n - 1)
    min_w = polyorder + 1
    if min_w % 2 == 0:
        min_w += 1
    window_len = max(window_len, min_w)

    coeffs = _savgol_coeffs(window_len, polyorder)

    def _sg_periodic(pts: np.ndarray) -> np.ndarray:
        # Pad by window_len//2 on each side so mode='valid' gives exactly n points.
        half = window_len // 2
        wx = np.concatenate([pts[-half:, 0], pts[:, 0], pts[:half, 0]])
        wy = np.concatenate([pts[-half:, 1], pts[:, 1], pts[:half, 1]])
        sx = np.convolve(wx, coeffs, mode='valid')
        sy = np.convolve(wy, coeffs, mode='valid')
        return np.column_stack([sx, sy])

    def _project(pts: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, int]:
        h, w = mask.shape
        cols = pts[:, 0].round().astype(int).clip(0, w - 1)
        rows = pts[:, 1].round().astype(int).clip(0, h - 1)
        outside = mask[rows, cols] == 0
        if outside.any():
            free_r, free_c = np.where(mask == 255)
            free_pts = np.column_stack([free_c, free_r]).astype(np.float64)
            for i in np.where(outside)[0]:
                j = np.argmin(np.sum((free_pts - pts[i]) ** 2, axis=1))
                pts[i, 0] = free_pts[j, 0]
                pts[i, 1] = free_pts[j, 1]
        return pts, int(outside.sum())

    # Pass 1: smooth → project overshoots back to the tight corridor
    pts = _sg_periodic(raw_pts)
    pts, n1 = _project(pts, constraint_mask)

    # Pass 2: re-smooth (removes kinks from projection) → project again
    # (re-smoothing can push a snapped point's neighbours outside the boundary)
    pts = _sg_periodic(pts)
    pts, n2 = _project(pts, constraint_mask)

    if n1 or n2:
        print(f"[centerline] Boundary projection: pass1={n1} pts, pass2={n2} pts snapped.")

    return pts


def _compute_wall_distances_lr(centerline_px, free_mask, resolution, yaw_world, max_dist_m=6.0):
    """
    Cast perpendicular rays right and left from each centerline point to measure
    the distance to the nearest wall in each direction.

    "Right" and "left" are relative to the direction of travel encoded in the
    ordering of centerline_px.  In pixel coords (col=x, row=y-down) the normals
    derived from world-frame yaw are:
        right: (dc, dr) = ( sin(yaw),  cos(yaw))
        left:  (dc, dr) = (-sin(yaw), -cos(yaw))
    """
    H, W = free_mask.shape
    max_px = int(max_dist_m / resolution) + 2
    right_m = np.full(len(centerline_px), max_dist_m)
    left_m  = np.full(len(centerline_px), max_dist_m)

    for i, (pt, yaw) in enumerate(zip(centerline_px, yaw_world)):
        col0, row0 = float(pt[0]), float(pt[1])
        dirs = [
            ( np.sin(yaw),  np.cos(yaw), right_m),
            (-np.sin(yaw), -np.cos(yaw), left_m),
        ]
        for dc, dr, out in dirs:
            for d in range(1, max_px):
                r = int(round(row0 + dr * d))
                c = int(round(col0 + dc * d))
                if r < 0 or r >= H or c < 0 or c >= W or free_mask[r, c] == 0:
                    out[i] = (d - 1) * resolution
                    break

    return right_m, left_m


# ── Public API ─────────────────────────────────────────────────────────────────

def extract_centerline_and_distances(
    yaml_path,
    map_path,
    max_track_width_m: float = 3.0,
    wall_margin_m: float = 0.2,
    point_spacing_m: float = 0.15,
    n_points_out: int = 1000,
):
    # Step 1: Load map
    grid_img, map_metadata = load_ros_map(yaml_path, map_path)
    resolution = map_metadata['resolution']

    # Step 2 & 3: Build free-space mask (handles both ROS and diagram-style maps)
    free_mask = build_free_mask(grid_img)

    # Step 4: Distance transform on the real free mask — for visualisation
    dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, maskSize=5)

    # Step 5: Inflate walls by wall_margin_m before computing the raw centerline.
    # Savitzky-Golay shortens corners geometrically; eroding the free space here
    # shifts the raw midpoints away from both walls by a uniform margin so that
    # after smoothing the result still clears the real walls.
    margin_px = max(1, round(wall_margin_m / resolution))
    erode_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * margin_px + 1, 2 * margin_px + 1)
    )
    free_mask_tight = cv2.erode(free_mask, erode_kernel)
    print(f"[centerline] Wall margin: {wall_margin_m} m ({margin_px} px per side)")

    # Step 6a: Coarse raw centerline at adaptive node density.
    # Spacing = point_spacing_m (default 0.15 m) ensures each node is well
    # above the ~0.5 px quantization noise floor of the contour extractor.
    coarse_centerline_px = extract_raw_centerline(
        free_mask_tight, resolution,
        point_spacing_m=point_spacing_m,
        max_track_width_m=max_track_width_m,
    )

    # Step 6b: Smooth the coarse nodes.
    # _auto_window_len adapts to the coarse spacing; two-pass SG with boundary
    # projection keeps the path inside the tight corridor.
    smoothed_coarse_px = smooth_centerline_closed(coarse_centerline_px, free_mask_tight)

    # Step 6c: Resample the smooth curve to the output density.
    # The smoothed coarse path is a clean geometric curve; arc-length
    # resampling gives uniform spacing with no staircase artifacts.
    smoothed_centerline_px = _resample_contour_arc_length(smoothed_coarse_px, n_points_out)
    raw_centerline_px = coarse_centerline_px

    distance_map_meters        = dist                    * resolution
    raw_centerline_meters      = raw_centerline_px       * resolution
    smoothed_centerline_meters = smoothed_centerline_px  * resolution

    return raw_centerline_meters, smoothed_centerline_meters, distance_map_meters, map_metadata


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args():
    parser = argparse.ArgumentParser(
        description="Extract the racetrack centerline from a ROS map YAML + image pair."
    )
    parser.add_argument('--yaml', type=str, default='map.yaml',
                        help='Path to the map metadata YAML file (default: map.yaml).')
    parser.add_argument('--map',  type=str, default='map.pgm',
                        help='Path to the map image (.pgm, .png, or any OpenCV-supported '
                             'format; default: map.pgm).')
    parser.add_argument('--max-track-width', type=float, default=3.0,
                        help='Maximum expected track width in metres (default: 3.0). '
                             'Inner-wall candidates farther than this from the outer wall '
                             'are rejected as infield regions or map artifacts.')
    parser.add_argument('--wall-margin', type=float, default=0.2,
                        help='Safety margin in metres to keep from each wall (default: 0.2). '
                             'The free space is eroded by this amount before computing the '
                             'raw centerline so that Savitzky-Golay corner-shortening does '
                             'not push the smoothed path into the walls.')
    parser.add_argument('--point-spacing', type=float, default=0.15,
                        help='Target physical spacing between raw centerline nodes in metres '
                             '(default: 0.15). Smaller values increase node density; larger '
                             'values reduce sensitivity to pixel quantization on tiny maps.')
    parser.add_argument('--output', type=str, default=None,
                        help='Path to save the output figure (e.g. centerline.png). '
                             'Defaults to <map_stem>_centerline.png next to the map image.')
    parser.add_argument('--output-csv', type=str, default=None,
                        help='Save the smoothed centerline to a CSV file with columns '
                             'x_meters,y_meters,w_dist_to_wall_right,w_dist_to_wall_left '
                             '(poses at ~10 cm arc-length spacing).')
    return parser.parse_args()


def main():
    args = _parse_args()
    print(f"Processing:\n  YAML: {args.yaml}\n  MAP:  {args.map}")

    try:
        raw_cl, centerline, dist_map, metadata = extract_centerline_and_distances(
            args.yaml, args.map,
            max_track_width_m=args.max_track_width,
            wall_margin_m=args.wall_margin,
            point_spacing_m=args.point_spacing,
        )

        print("\nDone.")
        print(f"Map resolution:              {metadata['resolution']} m/px")
        print(f"Centerline points (raw):      {len(raw_cl)}")
        print(f"Centerline points (smoothed): {len(centerline)}")

        grid_img, _ = load_ros_map(args.yaml, args.map)
        res = metadata['resolution']
        raw_px        = raw_cl     / res
        centerline_px = centerline / res
        dist_map_px   = dist_map   / res

        if args.output_csv is not None:
            # Resample to ~10 cm arc-length spacing before export.
            # Use a separate variable so the visualization below keeps the full
            # 1000-point smoothed path.
            diffs = np.diff(centerline, axis=0, append=centerline[0:1])
            arc_length_m = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))
            n_samples = max(2, round(arc_length_m / 0.10))
            csv_cl_px = _resample_contour_arc_length(centerline_px, n_samples)
            csv_cl    = csv_cl_px * res
            print(f"[centerline] CSV resampled to {n_samples} pts "
                  f"(arc={arc_length_m:.2f} m, spacing≈0.10 m)")

            # World-frame conversion:
            #   x_w = origin_x + col * res
            #   y_w = origin_y + (H_m - row * res)   (image row 0 = top, ROS Y grows up)
            H_m = grid_img.shape[0] * res
            ox, oy = metadata['origin'][0], metadata['origin'][1]
            world_x = ox + csv_cl[:, 0]
            world_y = oy + (H_m - csv_cl[:, 1])

            # Yaw from consecutive world-frame differences (wrap-around for closed loop)
            dx = np.diff(world_x, append=world_x[0:1])
            dy = np.diff(world_y, append=world_y[0:1])
            yaw_world = np.arctan2(dy, dx)

            free_mask = build_free_mask(grid_img)
            right_dist, left_dist = _compute_wall_distances_lr(
                csv_cl_px, free_mask, res, yaw_world
            )

            data = np.column_stack([world_x, world_y, right_dist, left_dist])
            np.savetxt(
                args.output_csv, data,
                delimiter=',',
                header='x_meters,y_meters,w_dist_to_wall_right,w_dist_to_wall_left',
                comments='',
            )
            print(f"Centerline CSV saved to: {args.output_csv}")

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

        # Left: map + raw midpoint contour + smoothed centerline
        ax1.imshow(grid_img, cmap='gray')
        ax1.scatter(raw_px[:, 0], raw_px[:, 1],
                    s=1, color='cyan', label='Raw centerline')
        ax1.plot(centerline_px[:, 0], centerline_px[:, 1],
                 color='red', linewidth=2, label='Smoothed centerline')
        ax1.set_title('Racetrack Map & Extracted Centerline')
        ax1.set_xlabel('X (px)')
        ax1.set_ylabel('Y (px)')
        ax1.legend(markerscale=6)
        ax1.set_aspect('equal')

        # Right: distance transform heatmap + raw + smoothed centerline
        dist_display = np.ma.masked_where(dist_map_px == 0, dist_map_px)
        im = ax2.imshow(dist_display, cmap='plasma', origin='upper')
        ax2.scatter(raw_px[:, 0], raw_px[:, 1],
                    s=1, color='cyan', label='Raw centerline')
        ax2.plot(centerline_px[:, 0], centerline_px[:, 1],
                 color='lime', linewidth=2, label='Smoothed centerline')
        fig.colorbar(im, ax=ax2, label='Distance to nearest wall (px)')
        ax2.set_title('Distance Transform')
        ax2.set_xlabel('X (px)')
        ax2.set_ylabel('Y (px)')
        ax2.legend(markerscale=6)
        ax2.set_aspect('equal')

        plt.tight_layout()

        output_path = args.output
        if output_path is None:
            stem = os.path.splitext(os.path.basename(args.map))[0]
            output_path = os.path.join(os.path.dirname(args.map) or '.', f"{stem}_centerline.png")
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {output_path}")

        print("Displaying visualisation — close the window to exit.")
        plt.show()

    except Exception as e:
        print(f"\nError: {e}")


if __name__ == '__main__':
    main()
