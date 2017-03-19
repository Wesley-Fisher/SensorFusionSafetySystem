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

class FakeRobot():

    def __init__(self):
        rospy.init_node('fake_robot')

        self.tf_time = 2.0
        self.tf = tf.TransformBroadcaster()

        self.process_queue = Queue.Queue()
    
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback=self.ModelCallback)


    def ModelCallback(self, msg):
        self.process_queue.put(msg)


    def MakeAllPoints(self):
        # Currently centered on post
        
        self.CreatePoint(0, 0, 0)        




    def ProcessModels(self, msg):
        if len(msg.name) < 1:
            return


        for n, p, t in zip(msg.name, msg.pose, msg.twist):

            if n == 'unit_sphere_1':
                x = p.position.x
                y = p.position.y
                z = p.position.z

                self.tf.sendTransform((x, y, z),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                'unit_sphere_1',
                "world")

                print("TF")
                


    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

            if not self.process_queue.empty():
                msg = self.process_queue.get()

                self.ProcessModels(msg)


if __name__ == "__main__":
    
    fr = FakeRobot()
    fr.main()