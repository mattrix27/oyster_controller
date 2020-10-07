#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import UInt16, Bool
from visualization_msgs.msg import Marker
from bag_detection.msg import FlipPos

class FlipFollower:

    IMU_TOPIC        = rospy.get_param("oyster_controller/imu")
    FLIP_CHECK_TOPIC = rospy.get_param("oyster_controller/flip_checker")
    FLIPPER_TOPIC    = rospy.get_param("oyster_controller/flipper_start")

    SPEED_TOL        = rospy.get_param("oyster_controller/speed_tolerance")
    COUNTER          = rospy.get_param("oyster_controller/counter")
    
    def __init__(self):
        rospy.Subscriber(self.FLIP_CHECK_TOPIC, BOOL, self.flip_check_callback)
        # rospy.Subscriber(self.IMU_TOPIC, UInt16, self.update_imu)
        self.flip_pub = rospy.Publisher(self.FLIPPER_TOPIC, Bool, queue_size=5)

	self.current_speed = 0
	self.flip_counter = 0


    def flip_check_callback(self, data):
	
        if data:
	    self.flip_counter += 1
	else:
	    self.flip_counter = 0
        
        if self.flip_counter > self.COUNTER:
	    self.flip_counter = 0
            self.flip_pub.publish(True)
	

if __name__ == "__main__":
    rospy.init_node('FlipFollower')
    flipFollower = FlipFollower()
    rospy.spin()
