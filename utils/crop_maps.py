#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import glob
import argparse
import cv2
import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

RAW_MAP = os.path.join(
    PACKAGE_ROOT,
    "maps",
    "raw",
    "surveillance_map_091423.pgm"
)

PREPROCESSED_MAP = os.path.join(
    PACKAGE_ROOT,
    "maps",
    "processed",
    "surveillance_map_preprocessed.pgm"
)

PLANNING_MAP = os.path.join(
    PACKAGE_ROOT,
    "maps",
    "processed",
    "surveillance_map_planning.pgm"
)

SCREENSHOTS_DIR = os.path.join(PACKAGE_ROOT, "docs", "screenshots")
OUTPUT_DIR = os.path.join(PACKAGE_ROOT, "docs", "report_figures")


def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def get_crop_bbox(mask, margin, width, height):
    ys, xs = np.where(mask)

    if len(xs) == 0 or len(ys) == 0:
        return None

    x_min = max(int(xs.min()) - margin, 0)
    x_max = min(int(xs.max()) + margin, width - 1)

    y_min = max(int(ys.min()) - margin, 0)
    y_max = min(int(ys.max()) + margin, height - 1)

    return x_min, y_min, x_max, y_max


def make_mask(img, mode):
    """
    mode:
      raw_map      - crop around known free/occupied space
      planning_map - crop around white free-space
      astar        - crop around white free-space and red path
    """

    if len(img.shape) == 2:
        gray = img
        free = gray > 240
        occupied = gray < 50

        if mode == "raw_map":
            # Raw gmapping map: unknown is grey, useful map is white/black.
            return np.logical_or(free, occupied)

        if mode == "planning_map":
            # Final planning map is mostly black, so crop around free space.
            return free

        return np.logical_or(free, occupied)

    # Colour image, usually A* debug output.
    b = img[:, :, 0]
    g = img[:, :, 1]
    r = img[:, :, 2]

    white_free = np.logical_and.reduce((r > 220, g > 220, b > 220))
    red_path = np.logical_and.reduce((r > 150, g < 120, b < 120))

    if mode == "astar":
        return np.logical_or(white_free, red_path)

    # Fallback: crop around bright non-black content.
    bright = np.logical_or.reduce((r > 220, g > 220, b > 220))
    return bright


def crop_and_save(input_path, output_path, mode, margin):
    if not os.path.exists(input_path):
        print("[SKIP] Missing:", input_path)
        return False

    img = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)

    if img is None:
        print("[SKIP] Could not read:", input_path)
        return False

    height, width = img.shape[:2]
    mask = make_mask(img, mode)

    bbox = get_crop_bbox(mask, margin, width, height)

    if bbox is None:
        print("[SKIP] No crop content found:", input_path)
        return False

    x_min, y_min, x_max, y_max = bbox
    cropped = img[y_min:y_max + 1, x_min:x_max + 1]

    ensure_dir(os.path.dirname(output_path))

    ok = cv2.imwrite(output_path, cropped)

    if ok:
        print("[OK] %s -> %s" % (input_path, output_path))
        print("     crop: x=%d:%d y=%d:%d size=%dx%d" % (
            x_min, x_max, y_min, y_max,
            cropped.shape[1],
            cropped.shape[0]
        ))
        return True

    print("[FAIL] Could not save:", output_path)
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Auto-crop maps and A* debug images for the report."
    )

    parser.add_argument(
        "--margin",
        type=int,
        default=80,
        help="Pixel margin around detected map content. Default: 80."
    )

    parser.add_argument(
        "--out",
        default=OUTPUT_DIR,
        help="Output directory. Default: docs/report_figures."
    )

    args = parser.parse_args()

    ensure_dir(args.out)

    # Map figures
    crop_and_save(
        RAW_MAP,
        os.path.join(args.out, "raw_map_cropped.png"),
        mode="raw_map",
        margin=args.margin
    )

    crop_and_save(
        PREPROCESSED_MAP,
        os.path.join(args.out, "preprocessed_map_cropped.png"),
        mode="planning_map",
        margin=args.margin
    )

    crop_and_save(
        PLANNING_MAP,
        os.path.join(args.out, "planning_map_cropped.png"),
        mode="planning_map",
        margin=args.margin
    )

    # A* debug figures
    astar_patterns = [
        os.path.join(SCREENSHOTS_DIR, "astar*.png"),
        os.path.join(SCREENSHOTS_DIR, "path*.png"),
    ]

    astar_files = []

    for pattern in astar_patterns:
        astar_files.extend(glob.glob(pattern))

    astar_files = sorted(list(set(astar_files)))

    if not astar_files:
        print("[INFO] No A* debug images found in:", SCREENSHOTS_DIR)

    for path in astar_files:
        name = os.path.splitext(os.path.basename(path))[0]
        output_path = os.path.join(args.out, name + "_cropped.png")

        crop_and_save(
            path,
            output_path,
            mode="astar",
            margin=args.margin
        )

    print("")
    print("Done. Cropped report figures saved to:")
    print("  %s" % args.out)
    print("")
    print("Use these only for the report figures, not for navigation/planning.")


if __name__ == "__main__":
    main()