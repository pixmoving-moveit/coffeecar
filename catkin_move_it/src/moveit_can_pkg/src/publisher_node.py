#!/usr/bin/env python
# license removed for brevity
import rospy
from can_msgs.msg import Frame
import numpy as np
speed_cmd = 10

def talker():
    pub = rospy.Publisher('sent_messages', Frame, queue_size=10)
    rospy.init_node('can_send', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        msg = Frame()
        data_list = [1, 255, 255, 1, 1, 1, 1]
        checksum = np.uint8(sum(data_list))
        print(checksum)
        data_list.append((3))

        msg.id = 0x0183
        msg.dlc = 8
        msg.data = data_list
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
