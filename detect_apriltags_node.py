#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagDetectionArray, BoolStamped
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class AprilPostPros(object):

    def __init__(self):    
        self.node_name = "apriltags_postprocessing_node"

        # -------- subscriber --------
        self.sub_prePros = rospy.Subscriber("~tag_info", AprilTagDetectionArray, self.callback, queue_size=1)

        # -------- publisher --------
        self.pub_info = rospy.Publisher("~info", BoolStamped, queue_size=1)

    def callback(self, msg):
        # Load tag detections message
        for detection in msg.detections:
            print (detection.id)

        
if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
