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
import sensor_msgs.point_cloud2
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from sf_msgs.msg import SFPoints
from sf_msgs.srv import NewGoal NewGoalRequest NewGoalResponse

import tf
import moveit_msgs
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import visualization_msgs

class LidarMonitor():

    def __init__(self):
        rospy.init_node('lidar_monitor')


        # Wait for MoveIt!
        self.last_scan = None
        self.have_new_scan = False
        self.processing = False

        self.timeout = 5.0

        self.laser_size = 500 # TODO


        self.kernel_size = 11 #make it odd
        self.offset = (self.kernel_size - 1)/2
        self.extra_skip = 5

        self.nan_reject_number = self.kernel_size*self.kernel_size/4 # Per kernel

        self.last_angle = 0

        self.goal_angle = 0

        self.angle_tol = 0.1


        self.point_sub = rospy.Subscriber('todo', LaserScan, callback=self.LaserCloudCallback)
        self.point_pub = rospy.Publisher('/observer_points_seen', SFPoints, queue_size=5)

        self.joint_sub = rospy.Subscriber('/joint_states', JointState, callback=JointCallback)

        self.goal_srv = rospy.ServiceProxy('/observer_goal_request', NewGoal)

        self.motor_pub = rospy.Publisher(TODO)

    def PointCloudCallback(self, msg):
        if not self.processing:
            self.last_scan = msg
            self.have_new_scan = True

    def JointCallback(self, msg):
        for n, p in zip(msg.name, msg.position):

            if n == 'xtion_tilt':
                self.last_tilt_angle = p

            if n == 'xtion_pan':
                self.last_pan_angle = p

    def CheckAngles(self):
        dp = self.last_pan_angle - self.goal_pan
        dt = self.last_tilt_angle - self.goal_tilt

        da = dp*dp + dt*dt

        if da < self.angle_tol:
            self.RequestNewGoal()

    def RequestNewGoal(self):

        req = NewGoalRequest()

        points = self.goal_srv(req)

        point = None

        for p in points.points:

            # Check reachability condition
            point = p
            break

        if point is not None:
            self.goal_angle = 0 TODO

            self.motor_pub.publish(self.goal_angle)



    def ProcessPoints(self):
        self.processing = True
        self.have_new_scan = False
        print("Processing")

        points_msg = SFPoints()

        points = []

        curr_angle = self.last_scan.angle_min


        for i in range(0, len(self.last_scan.ranges), self.kernel_size + self.extra_skip):

            avg_x = 0
            avg_y = 0
            avg_z = 0

            detected_nans = 0
            good_points = 0
            kernel_is_good = True

            for j in range(-self.offset, self.offset):

                r = self.last_scan.ranges[i+j]
                ang = self.last_scan.angle_min + (i+j)*self.last_scan.diff_angle

                x = r * math.cos(ang)
                y = r * math.sin(ang)
                z = 0

                if (x != x) or (y != y) or (z != z):
                    detected_nans = detected_nans + 1
                    if detected_nans > self.nan_reject_number:
                        kernel_is_good = False
                        break
                else:
                    good_points = good_points + 1

                    avg_x = avg_x + 1
                    avg_y = avg_y + 1
                    avg_z = avg_z + 1


            if kernel_is_good:
                pose = PoseStamped()
                pose.header.frame_id = 'laser'
                pose.pose.position.x = avg_x / good_points
                pose.pose.position.y = avg_y / good_points
                pose.pose.position.z = avg_z / good_points

                points_msg.points.append(pose)

        self.point_pub.publish(points_msg)
        self.processing = False


    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

            if self.have_new_scan:
                self.ProcessPoints()
                self.CheckAngles()


if __name__ == "__main__":
    
    l = LidarMonitor()
    l.main()