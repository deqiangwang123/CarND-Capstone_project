#!/usr/bin/env python
import numpy as np
from pid import PID
import lowpass
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
minimum_velocity = 0.1


class Controller(object):
    """
    acceleration and steering controller.
    acceleration is controlled via PID controller.
    Steering is calculated using YawController which simply calculates needed angle to keep needed velocity,
    after that steering is smoothed using low pass filter.
    """

    def __init__(self, vehicle_mass, fuel_capacity, accel_limit, deceleration_limit,
                 wheel_base, wheel_radius, steer_ratio,
                 max_lat_accel, max_steer_angle, min_speed):
        Kp = 1.5
        Ki = 0.001
        Kd = 0.
        self.velocity_controller = PID(kp=Kp, ki=Ki, kd=Kd, mn=deceleration_limit, mx=accel_limit)

        self.yaw_controller = YawController(wheel_base=wheel_base,
                                           steer_ratio=steer_ratio,
                                           min_speed=min_speed,
                                           max_lat_accel=max_lat_accel,
                                           max_steer_angle=max_steer_angle)
        self.wheel_radius = wheel_radius
        self.deceleration_limit = deceleration_limit
        self.mass = vehicle_mass +(fuel_capacity*GAS_DENSITY)

    def control(self, twist_cmd, current_velocity, time_span):
        """
        Calculates accel or deceleration and steering angle given current and requested velocity and time.
        In case accel is required it is returned as PID output, returned deceleration is zero.
        In case deceleration is required torque is calculated from PID output, returned accel is zero.
        :param twist_cmd: required velocity
        :param current_velocity: current velocity
        :param time_span: time passed from the last call
        :return: acceleration, deceleration, steering angle in wheel frame
        """
        velocity_error = twist_cmd.twist.linear.x - current_velocity.twist.linear.x
        accel = self.velocity_controller.step(velocity_error, time_span)
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
                                                 twist_cmd.twist.angular.z,
                                                 current_velocity.twist.linear.x)
        if np.isclose(twist_cmd.twist.linear.x, 0.) and current_velocity.twist.linear.x < minimum_velocity:
            return 0., self.calc_torque(self.deceleration_limit), steer
        else:
            if accel > 0:
                return accel, 0., steer
            else:
                torque = self.calc_torque(-accel, )
                return 0., torque, steer

    def calc_torque(self, accel):
        return accel * self.mass * self.wheel_radius

    def reset(self):
        self.velocity_controller.reset()
