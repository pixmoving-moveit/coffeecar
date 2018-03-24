#!/usr/bin/env python

from can_msgs.msg import Frame
from std_msgs.msg import Bool
from robot_coffee_msgs.msg import SpeedCmd, SteeringCmd, SteeringReport

import cantools
import numpy as np

import rospkg
import rospy

class CANPublisher(object):
    def __init__(self):
        rospy.init_node('can_send', anonymous=True)

        # Global variables
        self.speed_cmd = None
        self.steering_cmd = None
        self.gear_cmd = None
        self.dbw_enabled = None

        # Global constants
        self.dummy_constant = 1

        # Publishers
        self.can_pub = rospy.Publisher('/sent_messages', Frame, queue_size=10)

        # Subscribers
        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steering_cmd_cb)
        rospy.Subscriber('/vehicle/speed_cmd', SpeedCmd, self.speed_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        # Can database loading
        rospack = rospkg.RosPack()
        path = rospack.get_path('moveit_can_pkg')
        self.can_db = cantools.db.load_file(path + '/dbc/moveit_coffee.dbc')

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.can_talker()

            rate.sleep()

    def steering_cmd_cb(self, msg):
        self.steering_cmd = msg.steering_wheel_angle_cmd

    def speed_cmd_cb(self, msg):
        self.speed_cmd = msg.speed_cmd

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        if self.dbw_enabled:
            self.gear_cmd = 1
        else:
            self.gear_cmd = 1

    def can_talker(self):
        if (self.steering_cmd is not None and 
            self.speed_cmd is not None and 
            self.dbw_enabled is not None):

            msg = Frame()
            command_msg = self.can_db.get_message_by_name('Commands')
            msg.id = command_msg.frame_id
            msg.dlc = 8

            # Manual can encoding
            steering_cmd_raw = np.uint16((self.steering_cmd + 81.92)/0.005) #TODO do not hardcode constants
            steering_cmd_1 = int(np.uint8(np.uint16(steering_cmd_raw) >> 8))
            steering_cmd_2 = int(np.uint8(steering_cmd_raw))

            checksum = np.uint8(self.speed_cmd +
                                steering_cmd_1 +
                                steering_cmd_2 +
                                self.gear_cmd +
                                self.dummy_constant*3)

            data_list = [int(np.uint8(self.speed_cmd)), steering_cmd_1, steering_cmd_2, self.gear_cmd, 1, 1, 1, checksum]

            # Log Info
            rospy.loginfo(str(steering_cmd_raw))
            rospy.loginfo(str(data_list))

            msg.data = data_list

            self.can_pub.publish(msg)

if __name__ == '__main__':
    CANPublisher()
