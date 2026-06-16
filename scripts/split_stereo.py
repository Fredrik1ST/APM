#!/usr/bin/env python3
"""Split a stereo (side-by-side) image into separate left and right images.

Note: intended as a utility script for splitting images from the ZED Explorer.

For retrieving both images in real-time, it is probably faster to use the ZED SDK directly."""

import argparse
from pathlib import Path
import cv2


def split_stereo(input_path: Path, output_dir: Path | None = None) -> tuple[Path, Path]:
    """Split a stereo (side-by-side) image into separate right and left images.
    
    Usage:
        python split_stereo.py path/to/stereo_image.png -o path/to/output_dir
"""
    img = cv2.imread(str(input_path))
    if img is None:
        raise ValueError(f"Could not read image: {input_path}")

    mid = img.shape[1] // 2
    left = img[:, :mid]
    right = img[:, mid:]

    out_dir = output_dir or input_path.parent
    stem = input_path.stem
    suffix = input_path.suffix

    left_path = out_dir / f"{stem}_left{suffix}"
    right_path = out_dir / f"{stem}_right{suffix}"

    cv2.imwrite(str(left_path), left)
    cv2.imwrite(str(right_path), right)

    return left_path, right_path


def main():
    parser = argparse.ArgumentParser(description="Split stereo image into left/right halves.")
    parser.add_argument("images", nargs="+", type=Path, help="Input image(s) (.png or .jpg)")
    parser.add_argument("-o", "--output-dir", type=Path, default=None, help="Output directory (default: same as input)")
    args = parser.parse_args()

    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    for path in args.images:
        if path.suffix.lower() not in {".png", ".jpg", ".jpeg"}:
            print(f"Skipping {path}: unsupported format")
            continue
        left_path, right_path = split_stereo(path, args.output_dir)
        print(f"{path} -> {left_path}, {right_path}")


if __name__ == "__main__":
    main()
