#!/usr/bin/env python
from can_msgs.msg import Frame
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped

import cantools
import math

import rospy
import rospkg

#Global variables
wheel_angle = None
wheel_speed = None
wheel_angle_2 = None
DBW_mode = None

# Global Constants
wheel_base = 2.1 # in m

# Load can database
rospack = rospkg.RosPack()
path = rospack.get_path('moveit_can_pkg')
can_db = cantools.db.load_file(path + '/dbc/moveit_coffee.dbc')

# List of messages
feedback1_msg = can_db.get_message_by_name('Feedback_1')
feedback2_msg = can_db.get_message_by_name('Feedback_2')

def received_messages_cb(msg):
    msgID = msg.id

    if msgID == feedback1_msg.id:
        data = can_db.decode_message(msgID, msg.data)
        wheel_speed = data['Wheel_speed']
        wheel_angle = data['Wheel_Angle']

    if msgID == feedback2_msg.id:
        data = can_db.decode_message(msgID, msg.data)
        wheel_angle_2 = data['Wheel_Angle_2']
        DBW_mode = data['DBW_mode']

if __name__ == '__main__':
    # node init
    rospy.init_node('receive_can', anonymous=True)
    rate = rospy.Rate(10)

    # Subscribers
    rospy.Subscriber("/received_messages", Frame, received_messages_cb)

    # Publishers
    pub_actual_twist = rospy.Publisher("/actual_twist", TwistStamped, queue_size=1)
    pub_dbw_enabled = rospy.Publisher("/vehicle/actual_twist", Bool, queue_size=1)

    actual_twist_msg = TwistStamped()
    dbw_enabled_msg = Bool()

    while not rospy.is_shutdown():
        if (DBW_mode is not None and wheel_speed is not None):
            x_velocity = wheel_speed*0.277778
            actual_twist_msg.twist.linear.x = x_velocity

            wheel_angle_rad = wheel_angle*math.pi/180
            psi_rate = x_velocity*math.tan(wheel_angle_rad)/wheel_base
            actual_twist_msg.twist.angular.z = psi_rate

            if DBW_mode == 2:
                dbw_enabled_msg.data = True
            else:
                dbw_enabled_msg.data = False

            # Publish
            pub_actual_twist.publish(actual_twist_msg)
            pub_dbw_enabled.publish(dbw_enabled_msg)

        rate.sleep()
