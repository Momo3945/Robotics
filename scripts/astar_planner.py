#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import math
import heapq
import argparse
import yaml
import cv2
import numpy as np


# ------------------------------------------------------------
# Package-relative paths
# This assumes this file is:
# robot_assignment_ws/src/surveillance_bot/scripts/astar_planner.py
# ------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

MAPS_DIR = os.path.join(PACKAGE_ROOT, "maps")
RAW_MAP_YAML = os.path.join(MAPS_DIR, "raw", "surveillance_map_091423.yaml")
PROCESSED_MAP_YAML = os.path.join(MAPS_DIR, "processed", "surveillance_map_planning.yaml")

DOCS_DIR = os.path.join(PACKAGE_ROOT, "docs")
SCREENSHOTS_DIR = os.path.join(DOCS_DIR, "screenshots")
DEFAULT_DEBUG_PATH = os.path.join(SCREENSHOTS_DIR, "astar_debug_path.png")


class AStarPlanner(object):
    def __init__(self, yaml_path=None):
        if yaml_path is None:
            yaml_path = PROCESSED_MAP_YAML

        self.yaml_path = os.path.abspath(os.path.expanduser(yaml_path))

        if not os.path.exists(self.yaml_path):
            raise RuntimeError("Map YAML not found: %s" % self.yaml_path)

        with open(self.yaml_path, "r") as f:
            self.map_info = yaml.safe_load(f)

        image_path = self.map_info["image"]

        # If image path in YAML is relative, resolve it relative to the YAML folder.
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(self.yaml_path), image_path)

        self.image_path = os.path.abspath(os.path.expanduser(image_path))

        if not os.path.exists(self.image_path):
            raise RuntimeError("Map image not found: %s" % self.image_path)

        self.resolution = float(self.map_info["resolution"])
        self.origin_x = float(self.map_info["origin"][0])
        self.origin_y = float(self.map_info["origin"][1])

        img = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)

        if img is None:
            raise RuntimeError("Could not load map image: %s" % self.image_path)

        self.img = img
        self.height, self.width = img.shape

        # Planning map rule:
        # white = free, everything else = blocked.
        raw_free = img > 240

        # Extra safety clearance for the TurtleBot.
        # Map resolution is 0.05 m/pixel, so:
        # 5 pixels = 25 cm, 8 pixels = 40 cm, 10 pixels = 50 cm.
        # Increased from 7.5 to 10 to keep the path centre-line further from
        # walls, especially around tight corners.  The kernel size must be an
        # odd integer — passing a float to getStructuringElement silently
        # produces an incorrect kernel.
        SAFETY_RADIUS_PIXELS = 10
        kernel_size = int(2 * SAFETY_RADIUS_PIXELS + 1)   # must be int + odd

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size)
        )

        # Erode free space, which is equivalent to inflating obstacles.
        safe_free = cv2.erode(
            raw_free.astype(np.uint8),
            kernel,
            iterations=1
        )

        self.free = safe_free.astype(bool)
        print("[A*] Loaded map YAML: %s" % self.yaml_path)
        print("[A*] Loaded map image: %s" % self.image_path)
        print("[A*] Map size: %d x %d" % (self.width, self.height))
        print("[A*] Resolution: %.3f m/pixel" % self.resolution)
        print("[A*] Origin: x=%.3f, y=%.3f" % (self.origin_x, self.origin_y))

    def in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, cell):
        r, c = cell
        return self.in_bounds(cell) and self.free[r, c]

    def world_to_map(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row_from_bottom = int((y - self.origin_y) / self.resolution)
        row = self.height - 1 - row_from_bottom
        return (row, col)

    def map_to_world(self, cell):
        row, col = cell
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (self.height - row - 0.5) * self.resolution
        return (x, y)

    def heuristic(self, a, b):
        dr = a[0] - b[0]
        dc = a[1] - b[1]
        return math.sqrt(dr * dr + dc * dc)

    def neighbors(self, cell):
        r, c = cell

        directions = [
            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),
            (-1, -1, math.sqrt(2)),
            (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)),
            ( 1,  1, math.sqrt(2)),
        ]

        result = []

        for dr, dc, cost in directions:
            next_cell = (r + dr, c + dc)

            if not self.is_free(next_cell):
                continue

            # Prevent diagonal corner-cutting through walls.
            if dr != 0 and dc != 0:
                side1 = (r + dr, c)
                side2 = (r, c + dc)

                if not self.is_free(side1) or not self.is_free(side2):
                    continue

            result.append((next_cell, cost))

        return result

    def nearest_free(self, cell, max_radius=40):
        if self.is_free(cell):
            return cell

        r0, c0 = cell

        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue

                    candidate = (r0 + dr, c0 + dc)

                    if self.is_free(candidate):
                        print("[A*] Snapped blocked cell %s to nearest free cell %s" % (cell, candidate))
                        return candidate

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def astar(self, start, goal):
        start = self.nearest_free(start)
        goal = self.nearest_free(goal)

        if start is None:
            raise RuntimeError("Start is not near any free cell.")

        if goal is None:
            raise RuntimeError("Goal is not near any free cell.")

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed.add(current)

            for neighbor, move_cost in self.neighbors(current):
                if neighbor in closed:
                    continue

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))

        raise RuntimeError("No path found.")

    def line_is_free(self, a, b):
        r0, c0 = a
        r1, c1 = b

        dr = abs(r1 - r0)
        dc = abs(c1 - c0)

        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1

        err = dr - dc
        r, c = r0, c0

        while True:
            if not self.is_free((r, c)):
                return False

            if r == r1 and c == c1:
                break

            e2 = 2 * err

            if e2 > -dc:
                err -= dc
                r += sr

            if e2 < dr:
                err += dr
                c += sc

        return True

    def simplify_path(self, path):
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        i = 0

        while i < len(path) - 1:
            j = len(path) - 1

            while j > i + 1:
                if self.line_is_free(path[i], path[j]):
                    break
                j -= 1

            simplified.append(path[j])
            i = j

        return simplified

    def plan_world(self, start_x, start_y, goal_x, goal_y, simplify=True):
        start_cell = self.world_to_map(start_x, start_y)
        goal_cell = self.world_to_map(goal_x, goal_y)

        print("[A*] Start world: x=%.3f y=%.3f -> cell=%s" % (start_x, start_y, start_cell))
        print("[A*] Goal world:  x=%.3f y=%.3f -> cell=%s" % (goal_x, goal_y, goal_cell))

        cell_path = self.astar(start_cell, goal_cell)

        if simplify:
            cell_path = self.simplify_path(cell_path)

        world_path = [self.map_to_world(cell) for cell in cell_path]
        return world_path, cell_path

    def save_debug_path(self, cell_path, output_path=None):
        if output_path is None:
            output_path = DEFAULT_DEBUG_PATH

        output_path = os.path.abspath(os.path.expanduser(output_path))
        output_dir = os.path.dirname(output_path)

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        debug = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)

        # Draw thicker red dots/lines so they are visible.
        for i in range(len(cell_path)):
            r, c = cell_path[i]
            cv2.circle(debug, (c, r), 2, (0, 0, 255), -1)

            if i > 0:
                r_prev, c_prev = cell_path[i - 1]
                cv2.line(debug, (c_prev, r_prev), (c, r), (0, 0, 255), 1)

        ok = cv2.imwrite(output_path, debug)

        if not ok:
            raise RuntimeError("Failed to save debug path image: %s" % output_path)

        print("[A*] Saved debug path image: %s" % output_path)
        return output_path


def resolve_map_choice(choice):
    if choice == "raw":
        return RAW_MAP_YAML

    if choice == "processed":
        return PROCESSED_MAP_YAML

    raise RuntimeError("Unknown map choice: %s" % choice)


def main():
    parser = argparse.ArgumentParser(description="A* planner for SurveillanceBot.")

    parser.add_argument("start_x", type=float)
    parser.add_argument("start_y", type=float)
    parser.add_argument("goal_x", type=float)
    parser.add_argument("goal_y", type=float)

    parser.add_argument(
        "--map",
        choices=["processed", "raw"],
        default="processed",
        help="Which package map to use. Default: processed."
    )

    parser.add_argument(
        "--yaml",
        default=None,
        help="Optional explicit YAML path. Overrides --map."
    )

    parser.add_argument(
        "--no-simplify",
        action="store_true",
        help="Disable path simplification."
    )

    parser.add_argument(
        "--debug-out",
        default=DEFAULT_DEBUG_PATH,
        help="Where to save debug path image."
    )

    args = parser.parse_args()

    yaml_path = args.yaml if args.yaml else resolve_map_choice(args.map)

    planner = AStarPlanner(yaml_path)

    world_path, cell_path = planner.plan_world(
        args.start_x,
        args.start_y,
        args.goal_x,
        args.goal_y,
        simplify=not args.no_simplify
    )

    print("")
    print("[A*] Found path with %d waypoints:" % len(world_path))

    for i, point in enumerate(world_path):
        x, y = point
        print("  %02d: x=%.3f y=%.3f" % (i, x, y))

    planner.save_debug_path(cell_path, args.debug_out)


if __name__ == "__main__":
    main()