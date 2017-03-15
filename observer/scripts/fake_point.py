#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import Queue


from std_msgs.msg import Header
from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
import dynamixel_msgs
import sensor_msgs
from std_msgs.msg import ColorRGBA
from sf_msgs.msg import SFPoints

import tf
import moveit_msgs
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import visualization_msgs


rospy.init_node('fake_point')
time.sleep(1)

point_pub = rospy.Publisher('/observer_points_seen', SFPoints, queue_size=5)
time.sleep(1)

msg = SFPoints()

p = PoseStamped()
p.header.frame_id = 'xtion_camera_link'
p.pose.position.x = 0
p.pose.position.y = -3
p.pose.position.z = 0

msg.points.append(p)

point_pub.publish(msg)