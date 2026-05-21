#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import rospy

from astar_planner import AStarPlanner, PROCESSED_MAP_YAML
from controller import WaypointController


def read_goal_from_stdin():
    line = raw_input("\nEnter goal as: x y   or q to quit\n> ").strip()

    if line.lower() in ["q", "quit", "exit"]:
        return None

    parts = line.split()

    if len(parts) != 2:
        print("[NAV] Please enter exactly two numbers: x y")
        return "invalid"

    try:
        x = float(parts[0])
        y = float(parts[1])
        return (x, y)
    except ValueError:
        print("[NAV] Invalid numbers.")
        return "invalid"


def main():
    rospy.init_node("surveillance_navigator", anonymous=False)

    planner = AStarPlanner(PROCESSED_MAP_YAML)
    controller = WaypointController()

    rospy.loginfo("[NAV] Surveillance navigation node ready.")
    rospy.loginfo("[NAV] Type a target coordinate, for example: 3.0 4.0")

    while not rospy.is_shutdown():
        goal = read_goal_from_stdin()

        if goal is None:
            print("[NAV] Quitting.")
            controller.stop()
            break

        if goal == "invalid":
            continue

        goal_x, goal_y = goal

        try:
            start_x, start_y, start_yaw = controller.get_pose()

            print("")
            print("[NAV] Current robot pose:")
            print("      x=%.3f y=%.3f yaw=%.3f" % (start_x, start_y, start_yaw))
            print("[NAV] Goal:")
            print("      x=%.3f y=%.3f" % (goal_x, goal_y))

            # simplify=True uses line-of-sight to merge collinear/near-collinear
            # cells into fewer longer segments.  This pulls the path away from the
            # inflated obstacle boundary at corners instead of hugging it
            # cell-by-cell, which is what causes the robot to scrape walls.bugfix
            world_path, cell_path = planner.plan_world(
                start_x, start_y,
                goal_x, goal_y,
                simplify=True
            )

            timestamp = time.strftime("%H%M%S")
            debug_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "docs",
                "screenshots",
                "astar_path_%s.png" % timestamp
            )

            planner.save_debug_path(cell_path, debug_path)

            print("")
            print("[NAV] Planned path:")
            for i, point in enumerate(world_path):
                x, y = point
                print("      %02d: x=%.3f y=%.3f" % (i, x, y))

            print("")
            print("[NAV] Starting movement...")
            ok = controller.follow_path(world_path)

            if ok:
                print("[NAV] Goal reached.")
            else:
                print("[NAV] Navigation failed or timed out.")

        except Exception as e:
            controller.stop()
            print("[NAV] Error: %s" % str(e))


if __name__ == "__main__":
    main()