from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

class Controller(object):
    def __init__(self, *args, **kwargs):

        self.sample_time = kwargs['sample_time']
        self.min_speed = kwargs['min_speed']        # for the yaw controller
        self.max_speed = kwargs['max_speed']
        self.wheel_base = kwargs['wheel_base']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.linear_velocity_cmd = None
        self.angular_velocity_cmd = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.dbw_enabled = False

        self.yaw_controller = YawController(self.wheel_base, 1.0, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        self.lowpassfilter_steer_cmd = LowPassFilter(0.1, self.sample_time) # 0.1s time constant

    def control(self, *args, **kwargs):
        self.linear_velocity_cmd = kwargs['linear_velocity_cmd']
        self.angular_velocity_cmd = kwargs['angular_velocity_cmd']
        self.linear_velocity = kwargs['linear_velocity']
        self.angular_velocity = kwargs['angular_velocity']
        self.dbw_enabled = kwargs['dbw_enabled']

        if (self.dbw_enabled is True) and (self.linear_velocity_cmd is not None) and (self.linear_velocity is not None):
            speed_cmd = self.linear_velocity_cmd * 0.277778
            steer_cmd = self.yaw_controller.get_steering(self.linear_velocity_cmd, self.angular_velocity_cmd, self.linear_velocity)
        else:
            speed_cmd = 0.0
            steer_cmd = 0.0

        steer_cmd = self.lowpassfilter_steer_cmd.filt(steer_cmd)

        # commands limitation
        speed_cmd = max(self.min_speed, min(speed_cmd, self.max_speed))
        steer_cmd = max(-self.max_steer_angle, min(steer_cmd, self.max_steer_angle))

        if self.dbw_enabled is True:
            rospy.loginfo("speed_cmd: "+str(speed_cmd)+"\tsteer: "+str(steer_cmd))

        return speed_cmd, steer_cmd
