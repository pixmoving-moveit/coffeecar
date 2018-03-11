#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from robot_coffee_msgs.msg import SpeedCmd, SteeringCmd, SteeringReport
import math

from twist_controller import Controller

'''
This will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.

One thing to keep in mind is the status of `dbw_enabled`.
'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Parameters
        self.sample_time = rospy.get_param('~sample_time', 0.02)           # in s
        min_speed = rospy.get_param('~min_speed', 0.1)                     # in m/s
        max_speed = rospy.get_param('~max_speed', 0.1)                     # in m/s
        wheel_base = rospy.get_param('~wheel_base', 2.1)                   # in m
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)              # in m/s^2
        accel_limit = rospy.get_param('~accel_limit', 1.)                  # in m/s^2
        decel_limit = rospy.get_param('~decel_limit', -5)                  # in m/s^2
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 0.44)   # in rad
        self.direct_steering = rospy.get_param('~direct_steering', False)  # bool

        # Create `TwistController` object
        kwargs = {
                  'sample_time'       : self.sample_time,
                  'min_speed'         : min_speed,
                  'max_speed'         : max_speed,
                  'wheel_base'        : wheel_base,
                  'max_lat_accel'     : max_lat_accel,
                  'accel_limit'       : accel_limit,
                  'decel_limit'       : decel_limit,
                  'max_steer_angle'   : self.max_steer_angle,
        }
        self.controller = Controller(**kwargs)

        # Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.speed_pub = rospy.Publisher('/vehicle/speed_cmd', SpeedCmd, queue_size=1)

        # Subscribers
        self.twist_cmd = None
        self.current_velocity = None
        self.linear_velocity_cmd = None
        self.angular_velocity_cmd = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.dbw_enabled = False

        rospy.Subscriber('/actual_twist', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(1/self.sample_time) # 50Hz
        while not rospy.is_shutdown():
            kwargs = {
                      'linear_velocity_cmd'  : self.linear_velocity_cmd,
                      'angular_velocity_cmd' : self.angular_velocity_cmd,
                      'linear_velocity'      : self.linear_velocity,
                      'angular_velocity'     : self.angular_velocity,
                      'dbw_enabled'          : self.dbw_enabled
            }
            speed_cmd, steer_cmd = self.controller.control(**kwargs)

            if self.direct_steering is True and self.angular_velocity_cmd is not None :
                steer_cmd = self.angular_velocity_cmd/0.44 * self.max_steer_angle #TODO remove the hardcoded value (see line 26 teleop_joy.cpp)

            if (self.current_velocity is not None and self.twist_cmd is not None and self.dbw_enabled is not None):
                self.publish(speed_cmd, steer_cmd)

            rate.sleep()

    def publish(self, speed, steer):
        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        spdcmd = SpeedCmd()
        spdcmd.enable = True
        spdcmd.speed_cmd_type = SpeedCmd.CMD_KPH
        spdcmd.speed_cmd = speed
        self.speed_pub.publish(spdcmd)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg
        self.linear_velocity_cmd = self.twist_cmd.twist.linear.x
        self.angular_velocity_cmd = self.twist_cmd.twist.angular.z

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

if __name__ == '__main__':
    DBWNode()
