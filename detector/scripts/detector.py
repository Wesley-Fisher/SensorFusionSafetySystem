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

class Detector():

    def __init__(self):
        rospy.init_node('detector')

        self.tf_time = 2.0

        self.tf = tf.TransformListener(True, rospy.Duration(self.tf_time))
        time.sleep(self.tf_time)

        # If an observed point is close to this, stop the system
        self.coll_points = []
        self.coll_distances = []
        self.coll_ids = []

        # These are parts of hte system that might be observed
        self.clear_points = []
        self.clear_distances = []
        self.clear_ids = []

        self.frame = 'post_base'
        self.floor_limit = 0.1

        self.color_map = plt.get_cmap('jet')
        self.max_dist = 3.0


        self.process_queue = Queue.Queue()

        self.MakeAllPoints()
        print("Created Points")

        #self.vis_pub_clear = rospy.Publisher('/detector_clear_points', visualization_msgs.msg.MarkerArray,queue_size=5)
        #self.vis_pub_coll = rospy.Publisher('/detector_coll_points', visualization_msgs.msg.MarkerArray,queue_size=5)
        
        self.vis_pub = rospy.Publisher('/detector_signal', visualization_msgs.msg.Marker, queue_size = 5)
        self.point_sub = rospy.Subscriber('/observer_points_seen', SFPoints, callback=self.PointCallback)


    def PointCallback(self, msg):
        self.process_queue.put(msg)

    def MakeAllPoints(self):
        # Currently centered on post
        

        self.CreateCollPoint(0, 0, 0, 2)
        self.CreateClearPoint(0, 0, 0, 1, 'post_base')
        

    def CreateCollPoint(self, x, y, z, d):

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.frame

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z


        self.coll_points.append(p)
        self.coll_distances.append(d*d)

        if not self.coll_ids:
            self.coll_ids.append(1)
        else:
            self.coll_ids.append(max(self.coll_ids) + 1)

    def CreateClearPoint(self, x, y, z, d, frame):

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = frame

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z


        self.clear_points.append(p)
        self.clear_distances.append(d*d)

        if not self.clear_ids:
            self.clear_ids.append(1)
        else:
            self.clear_ids.append(max(self.clear_ids) + 1)


    def GetColor(self, d, d_coll):
        c = ColorRGBA()

        val = 1 / (1 + (d - d_coll))
        color = self.color_map(max(min(val/self.timeout, 0.99), 0.01))

        c.r = color[0]
        c.g = color[1]
        c.b = color[2]
        c.a = color[3]
        return c



    def RejectPoint(self, p, points, distances):
        # Return False if the point is a valid point
        # Return True if the point is part of the floor or a known part of the setup

        if p.pose.position.z < self.floor_limit:
            return True

        for point, d in zip(points, distances):

            dx = p.pose.position.x - point.pose.position.x
            dy = p.pose.position.y - point.pose.position.y
            dz = p.pose.position.z - point.pose.position.z

            dist = dx*dx + dy*dy + dx*dx

            if dist < d:
                return True

        return False

    def ProcessPoints(self, msg):
        time.sleep(0.1) # Try and wait for TFs

        source_frame = msg.points[0].header.frame_id
        scan_points = []
        self.tf.waitForTransform(self.frame, source_frame, time=rospy.Time.now(), timeout=rospy.Duration(0.5))
        for point in msg.points:
            p = self.tf.transformPose(self.frame, point)
            scan_points.append(p)

        clear_points = []
        for point in self.clear_points:
            p = self.tf.transformPose(self.frame, point)
            clear_points.append(p)
        
        for p, limit, index in zip(self.coll_points, self.coll_distances, self.coll_ids):
            px = p.pose.position.x
            py = p.pose.position.y
            pz = p.pose.position.z

            min_dist = 99999

            for tp in scan_points:

                if not self.RejectPoint(tp, clear_points, self.clear_distances):

                    tx = tp.pose.position.x
                    ty = tp.pose.position.y
                    tz = tp.pose.position.z

                    dx = px - tx
                    dy = py - ty
                    dz = pz - tz

                    d = dx*dx + dy*dy + dz*dz

                    if d < min_dist:
                        min_dist = d

            if min_dist <limit:
                self.TriggerCollision()

    def TriggerCollision(self):
        print("COLLISION!")

        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'detector_signal'
        marker.id = 0
        marker.lifetime = rospy.Duration(3.0)

        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.pose = geometry_msgs.msg.Pose()

        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2

        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1

        self.vis_pub.publish(marker)

    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

            if not self.process_queue.empty():
                msg = self.process_queue.get()

                try:
                    self.ProcessPoints(msg)
                except tf.ConnectivityException:
                    print("Connectivity Exception")
                except tf.Exception:
                    print("TF Exception")

if __name__ == "__main__":
    
    d = Detector()
    d.main()