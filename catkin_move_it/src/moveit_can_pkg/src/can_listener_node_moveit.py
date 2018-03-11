#!/usr/bin/env python
from can_msgs.msg import Frame
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

import cantools
import math

import rospy
import rospkg

class CANListener(object):
    def __init__(self):
        rospy.init_node('receive_can', anonymous=True)

        #Global variables
        self.wheel_angle = None
        self.wheel_speed = None
        self.wheel_angle_2 = None
        self.dbw_mode = None

        # Global Constants
        self.wheel_base = 2.1 # in m

        # Load can database
        rospack = rospkg.RosPack()
        path = rospack.get_path('moveit_can_pkg')
        self.can_db = cantools.db.load_file(path + '/dbc/moveit_coffee.dbc')

        # List of messages
        self.feedback1_msg = self.can_db.get_message_by_name('Feedback_1')
        self.feedback2_msg = self.can_db.get_message_by_name('Feedback_2')

        # Subscribers
        rospy.Subscriber("/received_messages", Frame, self.received_messages_cb)

        # Publishers
        self.pub_actual_twist = rospy.Publisher("/actual_twist", TwistStamped, queue_size=1)
        self.pub_dbw_enabled = rospy.Publisher("/vehicle/dbw_enabled", Bool, queue_size=1)

        # Published messages
        self.actual_twist_msg = TwistStamped()
        self.dbw_enabled_msg = Bool()

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            #rospy.loginfo("dbw_mode: " + str(self.dbw_mode))

            if (self.dbw_mode is not None and self.wheel_speed is not None):
                x_velocity = self.wheel_speed*0.277778
                self.actual_twist_msg.twist.linear.x = x_velocity

                wheel_angle_rad = self.wheel_angle*math.pi/180
                psi_rate = x_velocity*math.tan(wheel_angle_rad)/self.wheel_base
                self.actual_twist_msg.twist.angular.z = psi_rate

                if self.dbw_mode == 'Automatic':
                    self.dbw_enabled_msg.data = True
                else:
                    self.dbw_enabled_msg.data = False

                # Publish
                self.pub_actual_twist.publish(self.actual_twist_msg)
                self.pub_dbw_enabled.publish(self.dbw_enabled_msg)

            rate.sleep()

    def received_messages_cb(self, msg):
        msgID = msg.id

        if msgID == self.feedback1_msg.frame_id:
            data = self.can_db.decode_message(msgID, msg.data)
            self.wheel_speed = data['Wheel_speed']
            self.wheel_angle = data['Wheel_Angle']

        if msgID == self.feedback2_msg.frame_id:
            data = self.can_db.decode_message(msgID, msg.data)
            self.wheel_angle_2 = data['Wheel_Angle_2']
            self.dbw_mode = data['DBW_mode']

if __name__ == '__main__':
    CANListener()
