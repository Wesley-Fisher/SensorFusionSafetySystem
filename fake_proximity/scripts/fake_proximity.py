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
from sf_msgs.srv import NewGoal, NewGoalRequest, NewGoalResponse
#import gazebo_msgs.msg.ModelStates
from gazebo_msgs.msg import ModelStates

import tf
import sys
import copy
#import rospy
import geometry_msgs.msg
import visualization_msgs

class FakeProximity():

    def __init__(self):
        rospy.init_node('fake_proximity')

        self.tf_time = 2.0

        self.points = []

        self.frame = 'post_base'

        self.dist_threshold = 2.0
        self.velocity_threshold = 0.2

        self.process_queue = Queue.Queue()

        self.MakeAllPoints()
        print("Created Points")

        self.point_pub = rospy.Publisher('/observer_points_override', PoseStamped, queue_size=5)
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.ModelCallback)


    def ModelCallback(self, msg):
        self.process_queue.put(msg)


    def MakeAllPoints(self):
        # Currently centered on post
        
        self.CreatePoint(0, 0, 0)        



    def CreatePoint(self, x, y, z):

        t = rospy.Time.now()

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.frame

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z


        self.points.append(p)


    def ProcessModels(self, msg):
        if len(msg.name) < 1:
            return


        for p, t in zip(msg.pose, msg.twist):

            x = p.position.x
            y = p.position.y
            z = p.position.z

            for ir in self.points:

                dx = x - ir.pose.position.x
                dy = y - ir.pose.position.y
                dz = z - ir.pose.position.z

                dist = dx*dx + dy*dy + dz*dz

                

                if dist < self.dist_threshold:

                    vx = t.linear.x
                    vy = t.linear.y
                    vz = t.linear.z

                    vel = vx*vx + vy*vy + vz*vz

                    print("D, V = %f %f" % (dist, vel))

                    if vel > self.velocity_threshold:

                        self.point_pub.publish(ir)
                


    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

            if not self.process_queue.empty():
                msg = self.process_queue.get()

                self.ProcessModels(msg)


if __name__ == "__main__":
    
    fp = FakeProximity()
    fp.main()