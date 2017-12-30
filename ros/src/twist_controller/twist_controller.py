import math

from pid import PID
from lowpass import *
from yaw_controller import YawController

kp_v = 1.5
kd_v = 1.1
ki_v = 0.04

kp_s = 1.65
kd_s = 1.1
ki_s = 0.01


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
min_throttle = 0.0
max_throttle = 1.0


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_config = args[0]
        self.target_velocity = 0.0
        self.target_omega = 0.0
        self.current_velocity = 0.0
        self.dbw_status = False
        self.throttle = PID(kp_v,ki_v,kd_v,mn=min_throttle,mx=max_throttle)
        self.error_velocity = 0.0
        self.wheel_base = self.vehicle_config['wheel_base']
        self.steer_ratio = self.vehicle_config['steer_ratio']
        self.max_lat_accel = self.vehicle_config['max_lat_accel']
        self.max_steer_angle = self.vehicle_config['max_steer_angle']
        self.vehicle_mass = self.vehicle_config['vehicle_mass']
        self.wheel_radius = self.vehicle_config['wheel_radius']
        self.min_speed = 0.0

        self.steer = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
        self.steer_pid = PID(kp_s,ki_s,kd_s, mn=-self.max_steer_angle, mx=self.max_steer_angle)

        self.highest_velocity = 0.

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs

        target_velocity = args[0]
        target_omega = args[1]
        current_velocity = args[2]
        current_omega = args[3]
        dbw_status = args[4]
        dt = args[5]

        velocity_current = (abs(current_velocity.x))
        velocity_target = abs(target_velocity.x)
        velocity_error = velocity_target - velocity_current

        throttle_cmd = 0.0
        brake_cmd = 0.0

        brake_time = 2.8

        if (current_velocity.x<0.5) and (target_velocity.x < 0.2):
            throttle_cmd = 0.0
            self.throttle.reset()
            brake_cmd = (self.vehicle_mass * self.wheel_radius)/5

        else:
            throttle_cmd = self.throttle.step(velocity_error, dt)

            if (velocity_error < -0.15):
                brake_cmd = abs(self.vehicle_mass*self.wheel_radius*velocity_error/brake_time)

        omega_target = target_omega.z
        omega_current = 0.0
        steer_error = omega_target
        steer_cmd = self.steer.get_steering(velocity_target, steer_error, velocity_current)
        steer_cmd = self.steer_pid.step(steer_cmd,dt)

        return throttle_cmd, brake_cmd, steer_cmd
        
