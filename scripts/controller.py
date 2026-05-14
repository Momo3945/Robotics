#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState


class WaypointController(object):
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_mux/input/navi")
        self.model_name = rospy.get_param("~model_name", "mobile_base")

        self.max_linear = rospy.get_param("~max_linear", 0.22)
        self.max_angular = rospy.get_param("~max_angular", 0.75)

        self.k_linear = rospy.get_param("~k_linear", 0.45)
        self.k_angular = rospy.get_param("~k_angular", 1.60)

        self.waypoint_tolerance = rospy.get_param("~waypoint_tolerance", 0.18)
        self.final_tolerance = rospy.get_param("~final_tolerance", 0.22)

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)

        rospy.loginfo("[CTRL] Waiting for /gazebo/get_model_state...")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.loginfo("[CTRL] Controller ready.")
        rospy.loginfo("[CTRL] cmd_vel topic: %s" % self.cmd_topic)
        rospy.loginfo("[CTRL] model name: %s" % self.model_name)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def angle_wrap(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop(self):
        msg = Twist()
        self.pub.publish(msg)

    def get_pose(self):
        resp = self.get_model_state(self.model_name, "world")

        if not resp.success:
            raise RuntimeError("Could not get model state for: %s" % self.model_name)

        x = resp.pose.position.x
        y = resp.pose.position.y

        q = resp.pose.orientation

        # Since TurtleBot is mostly planar, yaw can be extracted from z/w.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return x, y, yaw

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def resample_path(self, world_path, max_step=0.45):
        """
        Add intermediate waypoints so the robot follows long A* segments smoothly.
        """
        if len(world_path) <= 1:
            return world_path

        result = [world_path[0]]

        for i in range(1, len(world_path)):
            x0, y0 = result[-1]
            x1, y1 = world_path[i]

            dx = x1 - x0
            dy = y1 - y0
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < 1e-6:
                continue

            steps = int(math.ceil(dist / max_step))

            for s in range(1, steps + 1):
                t = float(s) / float(steps)
                x = x0 + t * dx
                y = y0 + t * dy
                result.append((x, y))

        return result

    def drive_to_waypoint(self, target_x, target_y, is_final=False):
        rate = rospy.Rate(10)

        tolerance = self.final_tolerance if is_final else self.waypoint_tolerance

        start_time = rospy.Time.now().to_sec()
        timeout = 45.0

        while not rospy.is_shutdown():
            x, y, yaw = self.get_pose()

            dx = target_x - x
            dy = target_y - y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist <= tolerance:
                self.stop()
                return True

            target_yaw = math.atan2(dy, dx)
            angle_error = self.angle_wrap(target_yaw - yaw)

            # If badly misaligned, rotate first instead of driving forward.
            if abs(angle_error) > 0.45:
                linear = 0.0
            else:
                linear = self.clamp(self.k_linear * dist, 0.0, self.max_linear)

            angular = self.clamp(
                self.k_angular * angle_error,
                -self.max_angular,
                self.max_angular
            )

            self.publish_velocity(linear, angular)

            if rospy.Time.now().to_sec() - start_time > timeout:
                self.stop()
                rospy.logwarn("[CTRL] Timeout while driving to waypoint x=%.3f y=%.3f" % (target_x, target_y))
                return False

            rate.sleep()

        self.stop()
        return False

    def follow_path(self, world_path):
        if len(world_path) == 0:
            rospy.logwarn("[CTRL] Empty path.")
            return False

        path = self.resample_path(world_path, max_step=0.45)

        rospy.loginfo("[CTRL] Following %d waypoints after resampling." % len(path))

        # Skip the first point if it is basically the current robot position.
        start_index = 0
        robot_x, robot_y, _ = self.get_pose()
        first_x, first_y = path[0]
        first_dist = math.sqrt((first_x - robot_x) ** 2 + (first_y - robot_y) ** 2)

        if first_dist < 0.35:
            start_index = 1

        for i in range(start_index, len(path)):
            target_x, target_y = path[i]
            is_final = (i == len(path) - 1)

            rospy.loginfo("[CTRL] Waypoint %d/%d: x=%.3f y=%.3f" % (
                i + 1,
                len(path),
                target_x,
                target_y
            ))

            ok = self.drive_to_waypoint(target_x, target_y, is_final=is_final)

            if not ok:
                rospy.logwarn("[CTRL] Failed to reach waypoint.")
                return False

        self.stop()
        rospy.loginfo("[CTRL] Path complete.")
        return True