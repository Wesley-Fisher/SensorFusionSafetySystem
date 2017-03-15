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

class Observer():

    def __init__(self):
        rospy.init_node('observer')

        self.tf_time = 2.0

        self.tf = tf.TransformListener(True, rospy.Duration(self.tf_time))
        time.sleep(self.tf_time)

        self.points = []
        self.orientations = []
        self.times = []
        self.ids = []

        self.frame = 'post_base'

        self.length = 2.0 #forward, x
        self.width = 1.5 #side, y
        self.height = 1 # up, z

        self.num_length = 4
        self.num_width = 3
        self.num_height = 3

        self.color_map = plt.get_cmap('jet')

        self.timeout = 15.0

        self.angle_threshold = 0.2

        self.process_queue = Queue.Queue()

        self.MakeAllPoints()
        print("Created Points")

        self.vis_pub = rospy.Publisher('/observer_points', visualization_msgs.msg.MarkerArray,queue_size=5)
        self.vis_sensor_pub = rospy.Publisher('/sensor_points', visualization_msgs.msg.MarkerArray,queue_size=5)
        self.point_sub = rospy.Subscriber('/observer_points_seen', SFPoints, callback=self.PointCallback)


    def PointCallback(self, msg):
        self.process_queue.put(msg)

    def MakeAllPoints(self):
        # Currently centered on post
        

        # Make side walls
        for x in np.linspace(-self.length/2, self.length/2, self.num_length):
            for z in np.linspace(0, self.height, self.num_height):
                self.CreatePoint(x, self.width/2, z)
                self.CreatePoint(x, -self.width/2, z)

        # Make front/back walls
        for y in np.linspace(-self.width/2, self.width/2, self.num_width):
            for z in np.linspace(0, self.height, self.num_height):
                self.CreatePoint(self.length/2, y, z)
                self.CreatePoint(-self.length/2, y, z)

        # Make Ceiling
        for y in np.linspace(-self.width/2, self.width/2, self.num_width):
            for x in np.linspace(-self.length/2, self.length/2, self.num_length):
                self.CreatePoint(x, y, self.height)

        

    def CreatePoint(self, x, y, z):

        t = rospy.Time.now()

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.frame

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z

        yaw = math.atan2(y, x)
        r = math.sqrt(x*x + y*y)
        pitch = -math.atan2(z, r)

        quat = tf.transformations.quaternion_from_euler(0, pitch, yaw)
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]


        self.points.append(p)
        self.orientations.append((pitch, yaw))

        if not self.ids:
            self.ids.append(1)
        else:
            self.ids.append(max(self.ids) + 1)

        self.times.append(t)

    def GetColor(self, t, t_now):
        c = ColorRGBA()

        dt = (t_now - t).to_sec()
        color = self.color_map(min(dt/self.timeout, 0.99))

        c.r = color[0]
        c.g = color[1]
        c.b = color[2]
        c.a = color[3]
        return c

    def VisualizeGoalPoints(self):
        msg = visualization_msgs.msg.MarkerArray()
        t_now = rospy.Time.now()

        header = Header()
        header.frame_id = self.frame
        header.stamp = t_now

        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.05
        scale.y = 0.05
        scale.z = 0.05

        for p, t, i in zip(self.points, self.times, self.ids):
            marker = visualization_msgs.msg.Marker()
            marker.header = header
            marker.ns = 'observer_goal'
            marker.id = i
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.action = visualization_msgs.msg.Marker.ADD
            marker.pose = p.pose

            marker.scale = scale

            marker.color = self.GetColor(t, t_now)

            msg.markers.append(marker)

        self.vis_pub.publish(msg)

    def VisualizeSensorPoints(self, sensor_msg):
        msg = visualization_msgs.msg.MarkerArray()
        t_now = rospy.Time.now()

        header = Header()
        header.frame_id = sensor_msg.points[0].header.frame_id
        header.stamp = t_now

        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.01
        scale.y = 0.01
        scale.z = 0.01

        index = 0

        for p in sensor_msg.points:
            marker = visualization_msgs.msg.Marker()
            marker.header = header
            marker.ns = 'observer_sensor'
            marker.id = index
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.action = visualization_msgs.msg.Marker.ADD
            marker.pose = p.pose

            marker.scale = scale

            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 1

            msg.markers.append(marker)
            index = index + 1

        print('Visualizing Sensor')
        self.vis_sensor_pub.publish(msg)

    def AngleDiff(self, a, b):
        
        res = a - b

        #print("%f, %f => %f" % (a, b, res))
        return res

    def ProcessPoints(self, msg):
        target_frame = msg.points[0].header.frame_id

        temp_points = []
        self.tf.waitForTransform(target_frame, self.frame, time=rospy.Time.now(), timeout=rospy.Duration(0.5))
        for point in self.points:
            p = self.tf.transformPose(target_frame, point)
            temp_points.append(p)
        
        for p in msg.points:
            x = p.pose.position.x
            y = p.pose.position.y
            z = p.pose.position.z

            yaw = math.atan2(y, x)
            r = math.sqrt(x*x + y*y)
            pitch = -math.atan2(z, r)

            if (pitch != pitch) or (yaw != yaw):
                print(p)

            for index in range(0, len(self.points)):

                temp_position = temp_points[index].pose.position
                
                '''
                new_quat = (temp_points[index].pose.orientation.x,
                    temp_points[index].pose.orientation.y,
                    temp_points[index].pose.orientation.z,
                    temp_points[index].pose.orientation.w)
                
                temp_euler = tf.transformations.euler_from_quaternion(new_quat)
                temp_roll = 0#temp_euler[0]
                temp_pitch = -temp_euler[1]
                temp_yaw = temp_euler[2]
                '''

                temp_yaw = math.atan2(temp_position.y, temp_position.x)
                r = math.sqrt(temp_position.x*temp_position.x + temp_position.y*temp_position.y)
                temp_pitch = -math.atan2(temp_position.z, r)
                temp_roll = 0

                da = math.sqrt(temp_roll*temp_roll*0 + math.pow(self.AngleDiff(temp_pitch,pitch), 2) + math.pow(self.AngleDiff(temp_yaw, yaw), 2))


                if da < self.angle_threshold:
                    self.times[index] = rospy.Time.now()

                #print(da)

            '''
            quat = tf.transformations.quaternion_from_euler(0, pitch, yaw)
            p.pose.orientation.x = quat[0]
            p.pose.orientation.y = quat[1]
            p.pose.orientation.z = quat[2]
            p.pose.orientation.w = quat[3]

            q = p.pose.orientation * temp_points[i].inverse()
            '''



    def main(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

            if not self.process_queue.empty():
                msg = self.process_queue.get()

                try:
                    self.ProcessPoints(msg)
                    self.VisualizeSensorPoints(msg)
                except tf.ConnectivityException:
                    print("Connectivity Exception")
                except tf.Exception:
                    print("TF Exception")

            self.VisualizeGoalPoints()

if __name__ == "__main__":
    
    o = Observer()
    o.main()