"""
Pre-process diagram-style racetrack maps for the educational exercises.

Diagram maps have thin dark wall lines on a white background.  Both the
outer background and the infield are white, which makes them
indistinguishable from the drivable corridor under a simple > 240 threshold.

Fix (two passes):
  1. Outer background — any white connected component that touches the image
     border is outside the track.  Paint it mid-gray (127).
  2. Infield — after the outer background is removed, find the enclosed white
     region that has NO inner holes (the corridor ring always has the inner
     wall as a hole; the infield blob does not).  Paint it mid-gray too.

After both passes all map types share the same convention:
    white  (> 240) = drivable corridor only
    gray   (~127)  = outside / infield / unknown
    dark   (< 80)  = wall

Run once before starting the exercises:
    uv run preprocess_maps.py

Scans: scripts/f1tenth_racetracks/<Track>/<Track>_map.png
Output: alongside each original as  <Track>_map_processed.png
"""

import cv2
import glob
import numpy as np
import os

SCRIPT_DIR     = os.path.dirname(os.path.abspath(__file__))
RACETRACKS_DIR = os.path.join(SCRIPT_DIR, "..", "f1tenth_racetracks")


def paint_outer_background(img: np.ndarray, fill_value: int = 127) -> np.ndarray:
    """
    Replace every white connected component that touches the image border
    with `fill_value`.  Track corridor and infield (enclosed whites) are
    left unchanged.
    """
    _, white = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
    _, labels = cv2.connectedComponents(white)

    h, w = img.shape
    border_labels = (
        set(labels[0,  :].tolist()) |
        set(labels[-1, :].tolist()) |
        set(labels[:,  0].tolist()) |
        set(labels[:, -1].tolist())
    )
    border_labels.discard(0)   # label 0 is the dark background — keep it

    out = img.copy()
    for lbl in border_labels:
        out[labels == lbl] = fill_value
    return out


def paint_infield(img: np.ndarray, fill_value: int = 127) -> np.ndarray:
    """
    Replace the infield with `fill_value`.  Assumes the outer background
    has already been painted gray (by paint_outer_background).

    Algorithm
    ---------
    1. Close the dark wall lines to ensure the inner wall is a sealed ring
       (this image is used for analysis only, not written to the output).
    2. Build `white_closed`: pixels that are white in `img` AND not covered
       by the closed walls.  This separates the corridor ring from the infield
       blob even when the original wall lines have sub-pixel gaps.
    3. Run findContours with RETR_CCOMP on `white_closed`.
       - The track corridor is a ring: its level-0 contour HAS a level-1 child
         (the inner wall creates a topological hole in the corridor white region).
       - The infield is a simple blob: its level-0 contour has NO children.
    4. Fill every child-free top-level contour with `fill_value` (infield).
       Dark wall pixels inside the filled area are preserved.
    """
    # Step 1 — close wall gaps for topological analysis
    _, walls = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY_INV)
    close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    walls_closed = cv2.morphologyEx(walls, cv2.MORPH_CLOSE, close_k)

    # Step 2 — white pixels that are NOT covered by closed walls
    white_closed = np.where((img > 240) & (walls_closed == 0),
                            np.uint8(255), np.uint8(0))

    # Step 3 — contour hierarchy
    contours, hierarchy = cv2.findContours(
        white_closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )
    if hierarchy is None or len(contours) == 0:
        return img
    hierarchy = hierarchy[0]

    # Step 4 — paint child-free top-level contours (= infield blobs)
    out = img.copy()
    for i in range(len(contours)):
        is_top_level   = (hierarchy[i][3] == -1)   # no parent
        has_no_children = (hierarchy[i][2] == -1)   # no holes inside
        if is_top_level and has_no_children:
            mask = np.zeros(img.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, contours, i, 255, cv2.FILLED)
            # Paint only non-wall pixels (preserve dark wall lines)
            out[(mask == 255) & (img > 80)] = fill_value

    return out


def preprocess_map(img: np.ndarray, fill_value: int = 127) -> np.ndarray:
    """Full preprocessing pipeline: paint outer background then infield."""
    out = paint_outer_background(img, fill_value)
    out = paint_infield(out, fill_value)
    return out


def main():
    pattern = os.path.join(RACETRACKS_DIR, "*", "*_map.png")
    map_files = sorted(glob.glob(pattern))

    if not map_files:
        print(f"No map files found under: {RACETRACKS_DIR}")
        print("Expected pattern: f1tenth_racetracks/<Track>/<Track>_map.png")
        return

    print(f"Found {len(map_files)} map(s) to process.\n")

    for src in map_files:
        stem, ext = os.path.splitext(src)
        dst = f"{stem}_processed{ext}"

        img = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"  [skip] Cannot read: {src}")
            continue

        out = preprocess_map(img, fill_value=127)
        cv2.imwrite(dst, out)

        n_painted = int(np.sum(img > 240) - np.sum(out > 240))
        track_name = os.path.basename(src)
        print(f"  [ok]  {track_name:<40}  →  {os.path.basename(dst)}  "
              f"({n_painted:,} px painted gray)")

    print(f"\nDone. Processed files saved alongside their originals.")


if __name__ == "__main__":
    main()
