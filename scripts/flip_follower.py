#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import UInt16, Bool
from visualization_msgs.msg import Marker
from bag_detection.msg import FlipPos

class FlipFollower:

    BAG_POS_TOPIC    = rospy.get_param("oyster_controller/flip_pos_topic")
    DRIVE_TOPIC      = rospy.get_param("oyster_controller/flip_navigation")
    FLIP_ALIGN_TOPIC = rospy.get_param("oyster_controller/flip_align")
    MODE_TOPIC       = rospy.get_param("oyster_controller/mode")

    Kp               = rospy.get_param("oyster_controller/flip_Kp")
    Ki               = rospy.get_param("oyster_controller/flip_Ki")
    Kd               = rospy.get_param("oyster_controller/flip_Kd")

    MODE             = 0

    def __init__(self):
        rospy.Subscriber(self.BAG_POS_TOPIC, FlipPos, self.relative_pos_callback)
        rospy.Subscriber(self.MODE_TOPIC, Int32, self.update_mode)
        #self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, UInt16, queue_size=10)
        self.flip_pub = rospy.Publisher(self.FLIP_ALIGN_TOPIC, Bool, queue_size=5)

	    self.drive_mode = 0 #0: Free, 1: Incoming, 2: STOP


    def relative_pos_callback(self, data):
	
        if data.bot and data.top:
	        # IN THE ZONE, SEND STATUS TO FLIPPER CONTROLLER
            self.drive_mode = 2
            self.flip_pub.publish(True)

	    elif data.top_y < 480 and data.bot_y > 480:
            # FIRST HALF OF BAG DETECTED, SLOW DOWN MODE
            self.flip_pub.publish(False)
            # TODO: SLOWING DOWN DRIVE COMMAND
        else:
            # TODO: ALIGN TO BAG DRIVE COMMAND
            if self.MODE == 2:
                self.flip_pub.publish(True)
            else:
                self.flip_pub.publish(False)

	
    def update_mode(self, data):
        self.MODE = data

if __name__ == "__main__":
    rospy.init_node('FlipFollower')
    flipFollower = FlipFollower()
    rospy.spin()
