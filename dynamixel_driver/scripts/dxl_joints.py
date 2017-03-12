#!/usr/bin/env python
import rospy
import numpy as np
import math
import time


from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
import dynamixel_msgs
import sensor_msgs


import tf
import moveit_msgs
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class DXLJointManager():

    def __init__(self):
        rospy.init_node('dynamixel_manager')

        # Wait for MoveIt!
        time.sleep(5)

        

        $ dynmaixel_msgs/JointState

    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.5)

if __name__ == "__main__":
    
    d = DXLJointManager()
    d.main()