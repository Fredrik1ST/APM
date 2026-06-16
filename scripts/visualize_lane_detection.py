#!/usr/bin/env python3
"""Visualize lane detection pipeline step-by-step.

Saves (or displays) three images per input:
  1. <stem>_1_resized.jpg   - after crop + resize
  2. <stem>_2_canny.jpg     - Canny edges with mask applied
  3. <stem>_3_detected.jpg  - lane lines and heading arrow drawn on the resized image

Mask polygons are read from config/settings.toml (falls back to config/default.toml).

Usage:
    python scripts/visualize_lane_detection.py tests/input/img/frontcam/*.png --save tests/output/
    python scripts/visualize_lane_detection.py tests/input/img/frontcam/fwd_SVGA_left.png
    python scripts/visualize_lane_detection.py tests/input/img/frontcam/*.png --threshold 50 --no-mask
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import tomlkit

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from apm.config_handler import DEFAULT_CONFIG_PATH, SETTINGS_PATH
from apm.vision.lane_detector import LaneDetector
from apm.vision.frame_encoder import encode_lane_keeping_jpeg

_LABEL_FONT  = cv2.FONT_HERSHEY_SIMPLEX
_LABEL_SCALE = 0.8
_LABEL_COLOR = (200, 200, 200)


def load_config() -> tomlkit.TOMLDocument:
    path = SETTINGS_PATH if SETTINGS_PATH.exists() else DEFAULT_CONFIG_PATH
    return tomlkit.loads(path.read_text(encoding="utf-8"))


def label(image: np.ndarray, text: str) -> np.ndarray:
    out = image.copy()
    pos = (10, out.shape[0] - 12)
    cv2.putText(out, text, pos, _LABEL_FONT, _LABEL_SCALE, (0, 0, 0), 3)
    cv2.putText(out, text, pos, _LABEL_FONT, _LABEL_SCALE, _LABEL_COLOR, 1)
    return out


def to_bgr(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def run_pipeline(image_bgr: np.ndarray, detector: LaneDetector) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Run the full detection pipeline and return (resized, canny, annotated)."""
    resized = detector.resize_image(image_bgr)
    lanes   = detector.detect(resized)

    heading = None
    if lanes is not None:
        try:
            heading = lanes.get_heading()
        except ValueError:
            pass

    bgra      = cv2.cvtColor(resized, cv2.COLOR_BGR2BGRA)
    jpg_bytes = encode_lane_keeping_jpeg(
        bgra,
        lane_lines=lanes,
        heading_angle=heading,
        mask_polygons=detector.mask_polygons,
    )
    annotated = cv2.imdecode(np.frombuffer(jpg_bytes, np.uint8), cv2.IMREAD_COLOR)

    canny_bgr = to_bgr(detector.image_canny)

    status = f"lanes: {'OK' if lanes is not None else 'not detected'}"
    return (
        label(resized,   f"1 resized  {resized.shape[1]}x{resized.shape[0]}"),
        label(canny_bgr, f"2 canny+mask  {status}"),
        label(annotated, f"3 detected  hdg={heading:.1f} deg" if heading is not None else "3 detected  -"),
    )


def save_steps(stem: str, steps: tuple[np.ndarray, np.ndarray, np.ndarray], out_dir: Path):
    suffixes = ["_1_resized.jpg", "_2_canny.jpg", "_3_detected.jpg"]
    for img, suffix in zip(steps, suffixes):
        path = out_dir / f"{stem}{suffix}"
        cv2.imwrite(str(path), img)
        print(f"  {path}")


def display_steps(name: str, steps: tuple[np.ndarray, np.ndarray, np.ndarray]) -> bool:
    """Show each step in sequence. Returns True to continue, False to quit."""
    labels = ["resized", "canny+mask", "detected"]
    for img, lbl in zip(steps, labels):
        title = f"{name} - {lbl}  (any key: next | q: quit)"
        cv2.imshow(title, img)
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        if key == ord('q'):
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description="Visualize lane detection pipeline step by step.")
    parser.add_argument("images",       nargs="+", type=Path, help="Input image(s)")
    parser.add_argument("--cut-top",    type=float, default=None,  help="Fraction of image to discard from top (default: from config)")
    parser.add_argument("--threshold",  type=int,   default=None,  help="Hough accumulator threshold (default: from config)")
    parser.add_argument("--no-mask",    action="store_true",        help="Disable mask polygon from config")
    parser.add_argument("--save",       type=Path,  default=None,   help="Directory to save step images (default: display)")
    args = parser.parse_args()

    cfg = load_config()["lane_detector"]

    mask_polygons = None if args.no_mask else cfg["mask_polygons"]

    detector = LaneDetector(
        cut_top=args.cut_top if args.cut_top is not None else float(cfg["cut_top"]),
        angle_threshold=float(cfg["angle_threshold"]),
        hough_threshold=args.threshold if args.threshold is not None else int(cfg["hough_threshold"]),
        canny_sigma=float(cfg["canny_sigma"]),
        mask_polygons=mask_polygons,
    )

    if args.save:
        args.save.mkdir(parents=True, exist_ok=True)

    for path in args.images:
        img = cv2.imread(str(path))
        if img is None:
            print(f"Could not read {path}, skipping.")
            continue

        print(path.name)
        steps = run_pipeline(img, detector)

        if args.save:
            save_steps(path.stem, steps, args.save)
        else:
            if not display_steps(path.name, steps):
                break


if __name__ == "__main__":
    main()
