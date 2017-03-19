#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import Queue


import std_msgs
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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from sf_msgs.msg import SFPoints
from sf_msgs.srv import NewGoal, NewGoalRequest, NewGoalResponse

import tf
import moveit_msgs
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import visualization_msgs

class XtionMonitor():

    def __init__(self):
        rospy.init_node('xtion_monitor')

        self.loop_del = 0.2

        self.last_scan = None
        self.have_new_scan = False
        self.processing = False

        self.timeout = 5.0

        self.cloud_width = 320
        self.cloud_height = 240


        self.kernel_size = 11 #make it odd
        self.offset = (self.kernel_size - 1)/2
        self.extra_skip = 5

        self.nan_reject_number = self.kernel_size*self.kernel_size/4 # Per kernel


        self.point_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, callback=self.PointCloudCallback)
        self.point_pub = rospy.Publisher('/observer_points_seen', SFPoints, queue_size=5)


        self.joint_sub = rospy.Subscriber('/post/joint_states', JointState, callback=self.JointCallback)

        self.goal_srv = rospy.ServiceProxy('/observer_goal_request', NewGoal)

        self.last_pan = 0
        self.last_tilt = 0

        self.pan_goal = 0
        self.tilt_goal = 0

        self.angle_tol = 0.1

        self.pan_pub = rospy.Publisher('/post/pan_controller/command', std_msgs.msg.Float64, queue_size=5)
        self.tilt_pub = rospy.Publisher('/post/tilt_controller/command', std_msgs.msg.Float64, queue_size=5)


    def PointCloudCallback(self, msg):
        if not self.processing:
            self.last_scan = msg
            self.have_new_scan = True

    def JointCallback(self, msg):
        for n, p in zip(msg.name, msg.position):

            if n == 'xtion_pan':
                self.last_pan = p

            if n == 'xtion_tilt':
                self.last_tilt = p


    def CheckAngles(self):
        dt = self.last_tilt - self.tilt_goal
        dp = self.last_pan - self.pan_goal

        da = dp*dp +  dt*dt

        print(da)
        print("Pan (L,G) = %f, %f" % (self.last_pan, self.pan_goal))
        print("Tilt (L,G) = %f, %f" % (self.last_tilt, self.tilt_goal))


        if da < self.angle_tol:
            self.RequestNewGoal()

    def RequestNewGoal(self):

        req = NewGoalRequest()
        
        req.frame.data = 'xtion_base'

        #print(req)

        points = self.goal_srv(req)
        #print(points)

        #print(points)
        point = None

        for p in points.points.points:


            # Check reachability condition
            x = p.pose.position.x
            y = p.pose.position.y
            
            if x*x + y*y > 0.05:
                point = p
                break

        if point is not None:

            print(point)
            
            x = -p.pose.position.y
            y = p.pose.position.x
            z = p.pose.position.z

            r = math.sqrt(x*x + y*y)

            self.tilt_goal = math.atan2(z, r)
            self.pan_goal = math.fmod(math.atan2(y, x), 3.1415)

            print("New Goal")
            #print(point)
            print("P, T = %f, %f" % (self.pan_goal, self.tilt_goal))

            self.pan_pub.publish(self.pan_goal)
            self.tilt_pub.publish(self.tilt_goal)
        else:
            print("No New Goal")

    def GetIndicesInKernel(self, base_row, base_col):

        indices = []

        for drow in range(0, self.kernel_size):
            for dcol in range(0, self.kernel_size):

                row = base_row - self.offset + drow
                col = base_col - self.offset + dcol

                index = col + row*self.cloud_width

                indices.append(index)

        return indices

    def ProcessPoints(self):
        self.processing = True
        self.have_new_scan = False
        print("Processing")

        points_msg = SFPoints()

        points = []

        for point in sensor_msgs.point_cloud2.read_points(self.last_scan, skip_nans=False):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]

            point = Vector3()
            point.x = pt_x
            point.y = pt_y
            point.z = pt_z

            points.append(point)

        for i in range(self.offset, self.cloud_width - self.offset, self.kernel_size + self.extra_skip):
            for j in range(self.offset, self.cloud_height - self.offset, self.kernel_size + self.extra_skip):
                
                indices = self.GetIndicesInKernel(j, i)

                detected_nans = 0
                good_points = 1
                kernel_is_good = True
                avg_x = 0
                avg_y = 0
                avg_z = 0

                for index in indices:
                    
                    p = points[index]

                    if (p.x != p.x) or (p.y != p.y) or (p.z != p.z):
                        detected_nans = detected_nans + 1
                        if detected_nans > self.nan_reject_number:
                            kernel_is_good = False
                            break
                    else:
                        good_points = good_points + 1

                        avg_x = avg_x + p.x
                        avg_y = avg_y + p.y
                        avg_z = avg_z + p.z

                if kernel_is_good:

                    pose = PoseStamped()
                    pose.header.frame_id = 'camera_depth_optical_frame_virtual'
                    pose.pose.position.x = avg_x / good_points
                    pose.pose.position.y = avg_y / good_points
                    pose.pose.position.z = avg_z / good_points

                    points_msg.points.append(pose)


        self.point_pub.publish(points_msg)




        self.processing = False

    def main(self):
        self.pan_pub.publish(self.pan_goal)
        self.tilt_pub.publish(self.tilt_goal)

        while not rospy.is_shutdown():
            time.sleep(self.loop_del)
            self.pan_pub.publish(self.pan_goal)
            self.tilt_pub.publish(self.tilt_goal)

            if self.have_new_scan:
                self.ProcessPoints()
                self.CheckAngles()


if __name__ == "__main__":
    
    x = XtionMonitor()
    x.main()