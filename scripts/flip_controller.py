#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import UInt16, Bool
from visualization_msgs.msg import Marker
from bag_detection.msg import FlipPos

class BoatController:

    IMU_TOPIC        = rospy.get_param("oyster_controller/imu")
    BAG_POS_TOPIC    = rospy.get_param("oyster_controller/flip_pos_topic")
    FLIPPER_TOPIC    = rospy.get_param("oyster_controller/flipper_start")
    DRIVE_TOPIC      = rospy.get_param("oyster_controller/flip_navigation")
    MODE_TOPIC       = rospy.get_param("oyster_controller/mode")

    SPEED_TOL        = rospy.get_param("oyster_controller/speed_tolerance")
    COUNTER          = rospy.get_param("oyster_controller/counter")
    
    FLIP_ALIGN_TOPIC = rospy.get_param("oyster_controller/flip_align")
    MODE_TOPIC       = rospy.get_param("oyster_controller/mode")

    Kp               = rospy.get_param("oyster_controller/flip_Kp")
    Ki               = rospy.get_param("oyster_controller/flip_Ki")
    Kd               = rospy.get_param("oyster_controller/flip_Kd")

    def __init__(self):
        rospy.Subscriber(self.BAG_POS_TOPIC, FlipPos, self.flip_pos_callback)
        # rospy.Subscriber(self.IMU_TOPIC, UInt16, self.update_imu)
        # self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, UInt16, queue_size=5)
        self.flip_pub = rospy.Publisher(self.FLIPPER_TOPIC, Bool, queue_size=5)
        self.mode_pub = rospy.Publisher(self.MODE_TOPIC, UInt16, queue_size=5)

	    self.current_speed = 0
	    self.flip_counter = 0
        self.MODE = 0 #0: Align Bag, 1: FLipping, 2: FLip Check, 3. Next Bag, 4: Check Path, 5: Next Row


    def flip_pos_callback(self, data):

        # drive_msg = 
        if self.MODE == 4:
            self.MODE = 0
    
        if data.bot and data.top:
            # IN THE ZONE, SEND STATUS TO FLIPPER CONTROLLER
            self.flip_counter += 1

        elif data.top_y < 480 and data.bot_y > 480:
            # FIRST HALF OF BAG DETECTED, SLOW DOWN MODE
            self.flip_counter = 0
            # TODO: SLOWING DOWN DRIVE COMMAND
        else:
            # TODO: ALIGN TO BAG DRIVE COMMAND
            self.flip_counter = 0 

        # self.drive_pub(drive_msg)

        if self.flip_counter > 10 and self.current_speed < self.SPEED_TOL:
            self.flip_counter = 0
            self.MODE = 1
            self.flip_pub(True)




    def flip_check_callback(self, data):



    # def update_imu(self,data):
    #     self.current_speed = data


if __name__ == "__main__":
    rospy.init_node('FlipFollower')
    flipFollower = FlipFollower()
    rospy.spin()
