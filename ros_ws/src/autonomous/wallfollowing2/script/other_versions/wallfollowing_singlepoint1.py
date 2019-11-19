#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from drive_msgs.msg import drive_param

from rviz_geometry import show_circle_in_rviz, show_line_in_rviz

from circle import Circle, Point

import math

import numpy as np

from dynamic_reconfigure.server import Server
from wallfollowing2.cfg import wallfollowing2Config

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
#TOPIC_LASER_SCAN = "/scan" #for real lidar
TOPIC_LASER_SCAN = "/sim_scan"	
#for simulation data, the laser scan message in racer_plugins.gazebo is changed to /sim_scan

last_speed = 0


class Parameters():
    def __init__(self, default_values):
        self.names = default_values.keys()
        for name in self.names:
            setattr(self, name, default_values[name])

    def __str__(self):
        return '\n'.join(name + ": " + str(getattr(self, name))
                         for name in self.names)


class PIDController():
    def __init__(self, p, i, d, anti_windup=0.2):
        self.p = p
        self.i = i
        self.d = d
        self.anti_windup = anti_windup

        self.integral = 0
        self.previous_error = 0

    def update_and_get_correction(self, error, delta_time):
        self.integral += error * delta_time
        if self.integral > self.anti_windup:
            self.integral = self.anti_windup
        elif self.integral < -self.anti_windup:
            self.integral = -self.anti_windup

        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error
        return self.p * error + self.i * self.integral + self.d * derivative


def map(in_lower, in_upper, out_lower, out_upper, value):
    result = out_lower + (out_upper - out_lower) * \
        (value - in_lower) / (in_upper - in_lower)
    return min(out_upper, max(out_lower, result))


def drive(angle, velocity):
    message = drive_param()
    message.angle = angle
    message.velocity = velocity
    drive_parameters_publisher.publish(message)


def get_scan_as_cartesian(laser_scan):
    
    ranges = np.array(laser_scan.ranges)

    angles = np.linspace(
        laser_scan.angle_min,
        laser_scan.angle_max,
        ranges.shape[0])
    
    laser_range = laser_scan.angle_max - laser_scan.angle_min 
    #usable_range = math.radians(parameters.usable_laser_range)
    usable_range = math.radians(160)  #assume usable range 160 degree. To Do: add the cfg to cfg file.


    if usable_range < laser_range:
        skip_left = int((-laser_scan.angle_min - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        skip_right = int((laser_scan.angle_max - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        angles = angles[skip_left:-1 - skip_right]
        ranges = ranges[skip_left:-1 - skip_right]

    inf_mask = np.isinf(ranges)
    if inf_mask.any():
        ranges = ranges[~inf_mask]
        angles = angles[~inf_mask]

    #find angle of max range
    max_range = max(ranges)
    max_range_index = np.argmax(ranges)
    angle_max_range = angles[max_range_index]

    return max_range, angle_max_range


def control_commands(max_range, angle_max_range, delta_time):
    global last_speed
    speed_limit_acceleration = last_speed + parameters.max_acceleration * delta_time
    speed_max = 5
    speed = abs(speed_max * np.cos(angle_max_range))

    speed = min(speed_max, speed)

    steering_angle = parameters.high_speed_steering_limit * - np.sin(angle_max_range)
    print('speed_limit_acceleration', speed_limit_acceleration)
    print('steering_angle:', steering_angle)
    print('speed:', speed)
    drive(steering_angle, speed)


def handle_scan(laser_scan, delta_time):
    if parameters is None:
        return

    max_range, angle_max_range = get_scan_as_cartesian(laser_scan)
    control_commands(max_range, angle_max_range, delta_time)



last_scan = None


def laser_callback(scan_message):
    global last_scan

    scan_time = scan_message.header.stamp.to_sec()
    if last_scan is not None and abs(scan_time - last_scan) > 0.0001 and scan_time > last_scan:  # nopep8
        delta_time = scan_time - last_scan
        handle_scan(scan_message, delta_time)

    last_scan = scan_time


def dynamic_configuration_callback(config, level):
    global parameters
    parameters = Parameters(config)
    pid.p = parameters.controller_p
    pid.i = parameters.controller_i
    pid.d = parameters.controller_d
    return config


rospy.init_node('wallfollowing', anonymous=True)
parameters = None
pid = PIDController(1, 1, 1)

rospy.Subscriber(TOPIC_LASER_SCAN, LaserScan, laser_callback)
drive_parameters_publisher = rospy.Publisher(
    TOPIC_DRIVE_PARAMETERS, drive_param, queue_size=1)

Server(wallfollowing2Config, dynamic_configuration_callback)

while not rospy.is_shutdown():
    rospy.spin()
