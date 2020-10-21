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
    DRIVE_TOPIC      = rospy.get_param("oyster_controller/flip_navigation_topic")
    MODE_TOPIC       = rospy.get_param("oyster_controller/mode_topic")

    SPEED_TOL        = rospy.get_param("oyster_controller/speed_tolerance")
    FLIP_COUNT       = rospy.get_param("oyster_controller/flip_count")
    PATH_COUNT       = rospy.get_param("oyster_controller/path_count")   


    def __init__(self):
        rospy.Subscriber(self.BAG_POS_TOPIC, FlipPos, self.flip_pos_callback)
        rospy.Subscriber(self.FLIP_DONE_TOPIC, Bool, self.flip_done_callback)
        # rospy.Subscriber(self.IMU_TOPIC, UInt16, self.update_imu)


        # self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, UInt16, queue_size=5)
        self.flip_pub = rospy.Publisher(self.FLIP_START_TOPIC, Bool, queue_size=5)
        self.mode_pub = rospy.Publisher(self.MODE_TOPIC, UInt16, queue_size=5)

        self.current_speed = 0
        self.flip_counter = 0
        self.path_counter = 0
        self.MODE = 0 #0: Align Bag, 1: FLipping, 2: FLip Check, 3. Next Bag, 4: Check Path, 5: Next Row


    def flip_pos_callback(self, data):

        # drive_msg = 
        if self.MODE != 1:
            if data.top_y < 480 or data.bot_y < 480:
                self.path_counter = 0
                if self.MODE == 3 or self.MODE == 5: 
                    self.MODE = 0

                if self.MODE == 2:
                    self.flip_check(data)
                else:
                    self.bag_align(data)
            else:
                self.path_counter += 1
                if self.path_counter > self.PATH_COUNT:
                    self.path_counter = 0
                    self.MODE = 4
                    self.mode_pub.publish(self.MODE)
        

    def bag_align(self, data):
        # drive_msg =

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

        if self.flip_counter > self.FLIP_COUNT and self.current_speed < self.SPEED_TOL:
            print("FLIPPING")
            self.flip_counter = 0
            self.MODE = 1
            self.flip_pub.publish(True)

        # self.drive_pub.publish(drive_msg)


    def flip_check(self, data):
        if data.top_y < 480 and data.bot_y < 480:
            self.flip_counter += 1
        else:
            self.flip_counter = 0

        if self.flip_counter > 10 and self.current_speed < self.SPEED_TOL:
            print("FLIPPED")
            self.flip_counter = 0
            self.MODE = 3
            self.mode_pub.publish(self.MODE) 


    def flip_done_callback(self, data):
        print("CHECK FLIP")
        self.MODE = 2
        self.mode_pub.publish(self.MODE)


    # def update_imu(self,data):
    #     self.current_speed = data


if __name__ == "__main__":
    rospy.init_node('BoatController')
    BoatController = BoatController()
    rospy.spin()
