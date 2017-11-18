#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped, Point

class AprilFollow(object):

    def __init__(self):    
        self.node_name = "follow_apriltags_node"
        self.pose = Point()

        # -------- subscriber --------
        self.sub_pose = rospy.Subscriber("???", Point, self.callback, queue_size=1)

        # -------- publisher --------
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        print ("Start to follow apriltags:")

    def callback(self, msg):
        self.pose = msg
        self.car_cmd()
        self.stop()
        
    def car_cmd(self):
        cmd = Twist2DStamped()
        if self.pose.z > 0.15:
            cmd.v = 0.2
        elif self.pose.z < 0.20: 
            cmd.v = -0.2
        else:
            cmd.v = 0
            
        if self.pose.x > 0.02:
            cmd.omega = -1.8
        elif self.pose.x < -0.02:
            cmd.omega = 1.8
        else:
            cmd.omega = 0

        #publish the cmd
        ???

    # make the robot stop
    def stop(self):
        rospy.sleep(0.3)
        cmd = Twist2DStamped()
        cmd.v = 0
        cmd.omega = 0
        
        #publish the cmd
        ???

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilFollow()
    rospy.spin()