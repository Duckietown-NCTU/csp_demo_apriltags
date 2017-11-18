#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import ???
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped, Point

class AprilPostPros(object):

    def __init__(self):    
        self.node_name = "detect_apriltags_node"

        # -------- subscriber --------
        self.sub_prePros = rospy.Subscriber("???", AprilTagDetectionArray, self.callback, queue_size=1)

        # -------- publisher --------
        self.pub_info = rospy.Publisher("~position_info", Point, queue_size=1)

        print ("Start to detect apriltags:")

    def callback(self, msg):
        # Load tag detections message
        for detection in msg.detections:
            #Try to print the ID and position of the apriltags
            tag_id = ???
            x = ???
            y = ???
            z = ???
            print ("ID: ", tag_id)
            print ("(x,y,z): ", x, y, z)

            pos = Point()
            pos = detection.pose.pose.position
            self.pub_info.publish(pos)
        
if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()