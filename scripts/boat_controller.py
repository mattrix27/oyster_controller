#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import UInt16, Bool
from visualization_msgs.msg import Marker
from bag_detection.msg import FlipPos

class BoatController:

    IMU_TOPIC        = rospy.get_param("oyster_controller/imu_topic")
    BAG_POS_TOPIC    = rospy.get_param("oyster_controller/flip_pos_topic")
    FLIP_START_TOPIC = rospy.get_param("oyster_controller/flip_start_topic")
    FLIP_DONE_TOPIC  = rospy.get_param("oyster_controller/flip_done_topic")
    RC_COMMAND_TOPIC = rospy.get_param("oyster_controller/rc_command", "rc_command")
    DRIVE_TOPIC      = rospy.get_param("oyster_controller/flip_navigation_topic")
    MODE_TOPIC       = rospy.get_param("oyster_controller/mode_topic")

    SPEED_TOL        = rospy.get_param("oyster_controller/speed_tolerance")
    FLIP_COUNT       = rospy.get_param("oyster_controller/flip_count")
    PATH_COUNT       = rospy.get_param("oyster_controller/path_count")   


    def __init__(self):
        # rospy.Subscriber(self.BAG_POS_TOPIC, FlipPos, self.flip_pos_callback)
        rospy.Subscriber(self.FLIP_DONE_TOPIC, Bool, self.flip_done_callback)
        # rospy.Subscriber(self.IMU_TOPIC, UInt16, self.update_imu)
        rospy.Subscriber(self.RC_COMMAND_TOPIC, UInt16, self.rc_command_callback)  


        # self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, UInt16, queue_size=5)
        self.flip_pub = rospy.Publisher(self.FLIP_START_TOPIC, UInt16, queue_size=5)
        self.mode_pub = rospy.Publisher(self.MODE_TOPIC, UInt16, queue_size=5)

        self.current_speed = 0
        self.flip_counter = 0
        self.path_counter = 0
        self.MODE = 0 #0: IDLE, 1: FRONT, 2: ALIGN, 3. FLIP, 4: FLIPPED, 5: ROW CHECK 6. TURN


    def flip_pos_callback(self, data):
        if self.MODE == 2:
            if data.top_y < 480 or data.bot_y < 480:
                self.path_counter = 0
                self.bag_align(data)
        

    def bag_align(self, data):
        # drive_msg =

        if data.bot and data.top:
            # IN THE ZONE, SEND STATUS TO FLIPPER CONTROLLER
            self.flip_counter += 1

        elif data.top_y < 480 and data.bot_y > 480:
            # FIRST HALF OF BAG DETECTED, SLOW DOWN MODE
            self.flip_counter = 0
        else:
            # TODO: ALIGN TO BAG DRIVE COMMAND
            self.flip_counter = 0 

        if self.flip_counter > self.FLIP_COUNT and self.current_speed < self.SPEED_TOL:
            print("FLIPPING")
            self.flip_counter = 0
            self.MODE = 3
            self.mode_pub.publish(self.MODE)
            self.flip_pub.publish(0)

        # self.drive_pub.publish(drive_msg)


    def flip_done_callback(self, data):
        print("FLIPPED")
        self.MODE = 4
        self.mode_pub.publish(self.MODE)
	#TODO PULSE

    def rc_command_callback(self, data):
	if (self.MODE != 3):
            print("COMMAND: ", data.data)
            if (data.data == 3):
                self.flip_pub.publish(0)
            self.MODE = data.data
            self.mode_pub.publish(self.MODE)


    # def update_imu(self,data):
    #     self.current_speed = data


if __name__ == "__main__":
    rospy.init_node('BoatController')
    BoatController = BoatController()
    rospy.spin()
