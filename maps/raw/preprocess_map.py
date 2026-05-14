#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import cv2
import yaml
import numpy as np

# ------------------------------------------------------------
# This script is expected to be located in:
# robot_assignment_ws/src/surveillance_bot/maps/raw/
# ------------------------------------------------------------

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))          # maps/raw
MAPS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))       # maps
PROCESSED_DIR = os.path.join(MAPS_DIR, "processed")              # maps/processed

INPUT_YAML = os.path.join(SCRIPT_DIR, "surveillance_map_091423.yaml")
OUTPUT_BASENAME = os.path.join(PROCESSED_DIR, "surveillance_map_preprocessed")

# Tuning values
INFLATE_RADIUS_PIXELS = 4      # 4 pixels = about 20 cm if resolution is 0.05
MAX_UNKNOWN_FILL_AREA = 800    # only fill small grey blobs
FREE_BORDER_RATIO = 0.80       # grey blob must be mostly surrounded by white floor


def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def resolve_image_path(yaml_path, image_path):
    """
    Resolve the map image path.

    If the YAML contains an old absolute path, prefer a local image with the
    same filename in maps/raw/ if it exists.
    """
    image_name = os.path.basename(image_path)
    local_candidate = os.path.join(os.path.dirname(yaml_path), image_name)

    if os.path.exists(local_candidate):
        return local_candidate

    if os.path.isabs(image_path):
        return image_path

    return os.path.join(os.path.dirname(yaml_path), image_path)


def main():
    if not os.path.exists(INPUT_YAML):
        print("Could not find raw YAML:")
        print("  %s" % INPUT_YAML)
        sys.exit(1)

    ensure_dir(PROCESSED_DIR)

    data = load_yaml(INPUT_YAML)

    image_path = resolve_image_path(INPUT_YAML, data["image"])
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        print("Could not load map image:")
        print("  %s" % image_path)
        sys.exit(1)

    print("Loaded raw map:")
    print("  YAML:  %s" % INPUT_YAML)
    print("  Image: %s" % image_path)

    # ROS map_saver usually produces:
    # free ≈ 254, occupied ≈ 0, unknown ≈ 205
    free = img > 240
    occupied = img < 50
    unknown = np.logical_and(img >= 50, img <= 240)

    cleaned = img.copy()

    # ------------------------------------------------------------
    # Step 1: fill small unknown grey patches that are clearly floor
    # ------------------------------------------------------------
    unknown_u8 = unknown.astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(unknown_u8, 8)

    filled_count = 0

    for label in range(1, num_labels):
        area = stats[label, cv2.CC_STAT_AREA]

        if area > MAX_UNKNOWN_FILL_AREA:
            continue

        component = (labels == label).astype(np.uint8)

        # Make a one-pixel ring around the unknown component
        kernel3 = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(component, kernel3, iterations=1)
        ring = np.logical_and(dilated == 1, component == 0)

        ring_count = np.count_nonzero(ring)

        if ring_count == 0:
            continue

        free_ring_count = np.count_nonzero(np.logical_and(ring, free))
        ratio = float(free_ring_count) / float(ring_count)

        # Only fill unknown blobs mostly surrounded by known free space
        if ratio >= FREE_BORDER_RATIO:
            cleaned[labels == label] = 254
            filled_count += 1

    # ------------------------------------------------------------
    # Step 2: obstacle inflation
    # ------------------------------------------------------------
    occupied_clean = cleaned < 50

    radius = INFLATE_RADIUS_PIXELS
    kernel_size = 2 * radius + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernel_size, kernel_size)
    )

    inflated_obstacles = cv2.dilate(
        occupied_clean.astype(np.uint8),
        kernel,
        iterations=1
    )

    planning = cleaned.copy()

    # Remaining unknown is unsafe for planning, so make it blocked
    remaining_unknown = np.logical_and(planning >= 50, planning <= 240)
    planning[remaining_unknown] = 0

    # Inflated obstacles are blocked
    planning[inflated_obstacles == 1] = 0

    # Ensure free pixels are clean white
    planning[planning > 240] = 254

    output_pgm = OUTPUT_BASENAME + ".pgm"
    output_yaml = OUTPUT_BASENAME + ".yaml"

    ok = cv2.imwrite(output_pgm, planning)

    if not ok:
        print("Failed to save processed map:")
        print("  %s" % output_pgm)
        sys.exit(1)

    # Important:
    # Since the output YAML is saved inside maps/processed/,
    # the image path should be relative to maps/processed/.
    data["image"] = os.path.basename(output_pgm)

    with open(output_yaml, "w") as f:
        yaml.dump(data, f, default_flow_style=False)

    print("")
    print("Filled small unknown blobs: %d" % filled_count)
    print("Inflation radius: %d pixels" % INFLATE_RADIUS_PIXELS)
    print("")
    print("Saved preprocessed map:")
    print("  %s" % output_pgm)
    print("  %s" % output_yaml)
    print("")
    print("Next step:")
    print("  Manually clean this preprocessed map if needed, then save the final version as:")
    print("  %s" % os.path.join(PROCESSED_DIR, "surveillance_map_planning.pgm"))


if __name__ == "__main__":
    main()