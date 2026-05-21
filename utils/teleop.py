#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
teleop.py - Manual teleoperation with map saving
Controls:
  w - forward
  s - backward
  a - turn left
  d - turn right
  q - turn left (alias)
  e - turn right (alias)
  x - stop
  m - save map to ~/maps/surveillance_map
  SPACE - emergency stop
  ESC or ctrl+c - quit
"""
from gazebo_msgs.srv import GetModelState
import rospy
import sys
import tty
import termios
import os
import subprocess
import time
from geometry_msgs.msg import Twist
import math
CMD_TOPIC     = '/cmd_vel_mux/input/navi'
MAP_PATH      = os.path.expanduser('~/maps/surveillance_map')
FORWARD_SPEED = 0.30  # m/s
TURN_SPEED    = 0.50  # rad/s
BACK_SPEED    = 0.20  # m/s

rospy.init_node('teleop_mapper', anonymous=False)
pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

if not os.path.exists(os.path.dirname(MAP_PATH)):
    os.makedirs(os.path.dirname(MAP_PATH))

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def publish(linear, angular):
    t = Twist()
    t.linear.x  = linear
    t.angular.z = angular
    pub.publish(t)

def stop():
    publish(0.0, 0.0)

def save_map():
    stop()
    import time
    timestamp = time.strftime("%H%M%S")
    save_path = os.path.expanduser('~/maps/surveillance_map_%s' % timestamp)
    print("\n[MAP] Saving map to %s ..." % save_path)
    try:
        result = subprocess.call(
            ['rosrun', 'map_server', 'map_saver', '-f', save_path]
        )
        if result == 0:
            print("[MAP] Saved: %s.pgm + %s.yaml" % (save_path, save_path))
        else:
            print("[MAP] Save failed — is gmapping running?")
    except Exception as e:
        print("[MAP] Error: %s" % str(e))
    print("")

print("""

Tip: tap keys don't hold them  slow = better map
""")

try:
    while not rospy.is_shutdown():
        k = getkey()

        if k == 'w':
            publish(FORWARD_SPEED, 0.0)
            print(">> Forward")

        elif k == 's':
            publish(-BACK_SPEED, 0.0)
            print(">> Backward")

        elif k == 'a' or k == 'q':
            publish(0.0, TURN_SPEED)
            print(">> Turn Left")

        elif k == 'd' or k == 'e':
            publish(0.0, -TURN_SPEED)
            print(">> Turn Right")

        elif k == 'x' or k == ' ':
            stop()
            print(">> Stop")

        elif k == 'm':
            save_map()

        elif k == '\x1b' or k == '\x03':  # ESC or Ctrl+C
            print("\nQuitting...")
            stop()
            break
        elif k == 'p':
            try:
                get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                resp = get_state('turtlebot', 'world')
                x = resp.pose.position.x
                y = resp.pose.position.y
                q = resp.pose.orientation
                yaw = math.atan2(2*(q.w*q.z), 1 - 2*(q.z**2))
                print(">> Pose: x=%.3f y=%.3f yaw=%.3f" % (x, y, yaw))
            except Exception as e:
                print(">> Pose error: %s" % e)

        else:
            # Stop on any unrecognised key so robot doesn't keep moving
            stop()

except rospy.ROSInterruptException:
    pass
finally:
    stop()
    print("Teleop stopped.")