#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import UInt16
from visualization_msgs.msg import Marker
from bag_detection.msg import PathPos

class PathFollower:

    BAG_POS_TOPIC = rospy.get_param("oyster_controller/path_pos_topic")
    DRIVE_TOPIC   = rospy.get_param("oyster_controller/path_navigation")

    Kp            = rospy.get_param("oyster_controller/path_Kp")
    Ki            = rospy.get_param("oyster_controller/path_Ki")
    Kd            = rospy.get_param("oyster_controller/path_Kd")

    def __init__(self):
        rospy.Subscriber(self.BAG_POS_TOPIC, PathPos, self.relative_pos_callback)
        #self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, UInt16, queue_size=10)


    def relative_pos_callback(self, data):
	
        self.relative_x = data.x
        self.relative_y = data.y

	###A LOT TO BE DONE HERE 

        #self.drive_pub(10)
	

if __name__ == "__main__":
    rospy.init_node('PathFollower')
    pathFollower = PathFollower()
    rospy.spin()
