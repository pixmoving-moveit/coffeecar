#!/usr/bin/env python

'''
 file      can_listener_node.py
 authors   Mario Landry <mlandry@ivisolutions.ca>
           Ahmed
           Stanislav
           Johannes

'''

import math
import array
import rospy
import rospkg
import cantools
import numpy as np
from pprint import pprint

from std_msgs.msg import Bool, Int32MultiArray, Float32
from geometry_msgs.msg import TwistStamped
from can_msgs.msg import Frame


class CANListenerNode(object):
    def __init__(self):
        rospy.init_node('can_listener_node')

        # Load can database
        rospack = rospkg.RosPack()
        path = rospack.get_path('motrec_adas_can')
        self.can_db = cantools.db.load_file(path + '/dbc/motrec.dbc')

        # List of monitored can msgs
        self.canmsg_UserInput = self.can_db.get_message_by_name('UserInput') # TODO remove this line
        self.canmsg_Public01 = self.can_db.get_message_by_name('Public01')
        self.monitored_msgs_id = [self.canmsg_UserInput.frame_id, # TODO remove this line
                                  self.canmsg_Public01.frame_id]

        # List of published can msgs
        self.canmsg_KinectSensor01 = self.can_db.get_message_by_name('KinectSensor01')
        self.canmsg_KinectCluster01 = self.can_db.get_message_by_name('KinectCluster01')

        # Parameters
        zmax = rospy.get_param('~zmax', 12.0)                     # in m
        max_velocity = rospy.get_param('~max_velocity', 3.57)     # in m/s
        self.sample_time = rospy.get_param('~sample_time', 0.1)   # in s

        # Publishers
        self.velocity_pub = rospy.Publisher('/vehicle/act_velocity', TwistStamped, queue_size=1)
        self.steer_angle_pub = rospy.Publisher('/vehicle/act_steer_angle', Float32, queue_size=1)
        self.can_pub = rospy.Publisher('/can/sent_messages', Frame, queue_size=10)

        # Subscribers
        rospy.Subscriber('/free_dist', Float32, self.free_dist_ahead_cb)   #TODO change topic name..
        rospy.Subscriber('/camera/speed_limit', Float32, self.speed_limit_cb)
        rospy.Subscriber('/camera/hmi_pix_matrix', Int32MultiArray, self.hmi_pix_matrix_cb)
        rospy.Subscriber('/received_messages', Frame, self.can_read_cb)

        # Attributes
        self.act_velocity = None
        self.act_steer_angle = None
        self.act_speed_limit = None
        self.act_free_dist_ahead = None
        self.hmi_pix_matrix = None
        self.reset_signal = None       #TODO remove
        self.adas_enable = None        #TODO remove
        self.speed_limit_enable = None #TODO remove

        self.loop()

    def loop(self):
        rate = rospy.Rate(1/self.sample_time) # 200Hz
        while not rospy.is_shutdown():
            
        #    rospy.loginfo("reset: " + str(self.reset_signal) + 
        #                  " adas enable: " + str(self.adas_enable) + 
        #                  " speed limit enable : " + str(self.speed_limit_enable))
            self.publish()
            rate.sleep()

    def publish(self):
        if self.act_velocity is not None:
            vel = TwistStamped()
            vel.header.frame_id = '/world'
            vel.header.stamp = rospy.Time(0)
            vel.twist.linear.x = self.act_velocity
            self.velocity_pub.publish(vel)

        if self.act_steer_angle is not None:
            steer = Float32()
            steer.data = self.act_steer_angle
            self.steer_angle_pub.publish(steer)

        if (self.act_free_dist_ahead is not None) and (self.act_speed_limit is not None):
            frame_KinectSensor01 = Frame()
            frame_KinectSensor01.header.stamp = rospy.Time(0)
            frame_KinectSensor01.id = self.canmsg_KinectSensor01.frame_id
            frame_KinectSensor01.data = self.canmsg_KinectSensor01.encode({'SpeedLimit': self.act_speed_limit,
                                                                           'EStopReq': 0,
                                                                           'WarningZoneStatus': 0,
                                                                           'FreeDistance': self.act_free_dist_ahead,
                                                                           'KinectError': 0})
            self.can_pub.publish(frame_KinectSensor01)
        
        if self.hmi_pix_matrix is not None:
            frame_KinectCluster01 = Frame()
            frame_KinectCluster01.header.stamp = rospy.Time(0)
            frame_KinectCluster01.id = self.canmsg_KinectCluster01.frame_id
            # Dictionnary construction
            dictionnary = {}
            for i in range(5):
                for j in range(5):
                    key = "Pixel"+str(i)+"_"+str(j)
                    dictionnary[key] = self.hmi_pix_matrix[i*5+j]

        frame_KinectCluster01.data = self.canmsg_KinectCluster01.encode(dictionnary)
        self.can_pub.publish(frame_KinectCluster01)


    def free_dist_ahead_cb(self, msg):
        self.act_free_dist_ahead = msg.data

    def speed_limit_cb(self, msg):
        self.act_speed_limit = msg.data

    def hmi_pix_matrix_cb(self, msg):
        self.hmi_pix_matrix = msg.data

    def can_read_cb(self, msg):
        stamp = msg.header.stamp
        id = msg.id
        is_extended = msg.is_extended
        is_error = msg.is_error
        dlc = msg.dlc

        msg_to_decode = False
        for k in self.monitored_msgs_id:
            if id == k:
                msg_to_decode = True

        if msg_to_decode is True:
            data = self.can_db.decode_message(id, msg.data)

        if id == self.canmsg_Public01.frame_id:
            self.act_steer_angle = data['ActSteeringAngle']
            self.act_velocity = data['ActSpeed']

        if id == self.canmsg_UserInput.frame_id:                  #TODO remove
            self.reset_signal = data['Reset_Signal']              #TODO remove
            self.adas_enable = data['ADAS_Enable']                #TODO remove
            self.speed_limit_enable = data['Speed_Limit_Enable']  #TODO remove

if __name__ == '__main__':
    CANListenerNode()
