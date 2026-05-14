#!/usr/bin/env python
import rospy, math
from gazebo_msgs.srv import GetModelState

rospy.wait_for_service('/gazebo/get_model_state')
_get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def get_pose():
    resp = _get_state('mobile_base', 'world')
    x = resp.pose.position.x
    y = resp.pose.position.y
    q = resp.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
    return x, y, yaw