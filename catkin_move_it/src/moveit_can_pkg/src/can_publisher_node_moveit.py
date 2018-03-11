#!/usr/bin/env python
# license removed for brevity
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
import cantools
import numpy as np
import math

speed_cmd = 0
steering_cmd = 0
gear_cmd = 0
flag_cmd = 0
car_length = 4 # should be changed

def twist_cb(msg):
    global flag_cmd
    flag_cmd = 1
    global speed_cmd
    speed_cmd = msg.linear.x*3.6
    global steering_cmd
    steering_cmd = math.atan(((msg.angular.z*car_length)/speed_cmd))



def talker():
    pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, twist_cb)
    rospy.init_node('can_send', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if (flag_cmd == 1):
            msg = Frame()
            command_msg = can_db.get_message_by_name('Commands')
            msg.id = command_msg.frame_id
            checksum = np.uint8(speed_cmd + steering_cmd + gear_cmd + 1 + 1 + 1)
            msg.data = command_msg.encode({'SpdCmd': speed_cmd,
                                           'SteeringCmd': steering_cmd,
                                           'GearCmd': gear_cmd,
                                           'Checksum': checksum})
            pub.publish(msg)

        rate.sleep()
        # data_list = [1, 255, 255, 1, 1, 1, 1]

        # print(checksum)
        # data_list.append(3)

        # msg.id = 391 #0x0183
        # msg.dlc = 8
        # msg.data = data_list


if __name__ == '__main__':
    can_db = cantools.db.load_file('./dbc/moveit_coffee.dbc')
    talker()
