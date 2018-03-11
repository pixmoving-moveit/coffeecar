#!/usr/bin/env python

from can_msgs.msg import Frame
from std_msgs.msg import Bool
from robot_coffee_msgs.msg import SpeedCmd, SteeringCmd, SteeringReport

import cantools
import numpy as np

import rospkg
import rospy

# Global variables
speed_cmd = None
steering_cmd = None
gear_cmd = None
dbw_enabled = None

# Global constants
dummy_constant = 1
car_length = 2.1   # in m

def steering_cmd_cb(msg):
    steering_cmd = msg.steering_wheel_angle_cmd

def speed_cmd_cb(msg):
    speed_cmd = msg.speed_cmd

def dbw_enabled_cb(msg):
    dbw_enabled = msg.data
    if dbw_enabled:
        gear_cmd = 2
    else:
        gear_cmd = 1

def can_talker():
    if (steering_cmd is not None and speed_cmd is not None and dbw_enabled is not None):
        msg = Frame()
        command_msg = can_db.get_message_by_name('Commands')
        msg.id = command_msg.frame_id
        checksum = np.uint8(speed_cmd + steering_cmd + gear_cmd + dummy_constant*3)
        msg.data = command_msg.encode({'SpdCmd': speed_cmd,
                                       'SteeringCmd': steering_cmd,
                                       'GearCmd': gear_cmd,
                                       'something_01_1': dummy_constant,
                                       'something_01_2': dummy_constant,
                                       'something_01_3': dummy_constant,
                                       'Checksum': checksum})
        can_pub.publish(msg)

if __name__ == '__main__':

    # Publishers
    can_pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, steering_cmd_cb)
    rospy.Subscriber('/vehicle/speed_cmd', SpeedCmd, speed_cmd_cb)
    rospy.Subscriber('/vehicle/dbw_enabled', Bool, dbw_enabled_cb)

    # Can database loading
    rospack = rospkg.RosPack()
    path = rospack.get_path('moveit_can_pkg')
    can_db = cantools.db.load_file(path + '/dbc/moveit_coffee.dbc')

    # ROS node init
    rospy.init_node('can_send', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        can_talker()

        rate.sleep()
