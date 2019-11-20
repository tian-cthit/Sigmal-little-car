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
from statistics import mean
import time


TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_LASER_SCAN = "/sim_scan"

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
    ranges = ranges[0::6]  # get 1 point out of every 6 points, final length of ranges: 180
    angles = np.linspace(
        laser_scan.angle_min,
        laser_scan.angle_max,
        ranges.shape[0])

    laser_range = laser_scan.angle_max - laser_scan.angle_min
    usable_range_angle = 180
    usable_range = math.radians(usable_range_angle)
    if usable_range < laser_range:
        skip_left = int((-laser_scan.angle_min - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        skip_right = int((laser_scan.angle_max - usable_range / 2) / laser_range * ranges.shape[0])  # nopep8
        angles = angles[skip_left:-1 - skip_right]
        ranges = ranges[skip_left:-1 - skip_right]

    inf_mask = np.isinf(ranges)
    if inf_mask.any():
        ranges = ranges[~inf_mask]
        angles = angles[~inf_mask]

    points = np.zeros((ranges.shape[0], 2))
    points[:, 0] = -np.sin(angles) * ranges
    points[:, 1] = np.cos(angles) * ranges

    angle_max_range = greedy_exploration(points, ranges, usable_range_angle)

    return points, angle_max_range


def speed_limit_obstacles(points, ranges, number_of_detections):

    pass


def greedy_exploration(points, ranges, usable_range_angle):
    length = len(ranges)
    bodysizeb = 0.18 + 0.5  # width of car + extra safe margin

    # search by gates
    safe_margin = 0.8
    gate_angle = 5  # degrees, search every * degrees
    number_of_detections = usable_range_angle//gate_angle - 1
    detection = [[0] for k in range(number_of_detections)]  # laser points in each gate

    for i in range(number_of_detections):  # only consider the conditions: usable_range_angle <=180 degrees

        gate_rad = math.radians(-usable_range_angle/2 + (i+1)*gate_angle)

        # points in each gate
        points_x_plus_y_side = points[:, 1] + points[:, 0]/np.tan(gate_rad)    # y + x/-tan(-theta)
        points_x_plus_y_bottom = points[:, 1] - points[:, 0]*np.tan(gate_rad)

        if gate_rad < 0:
            for j in range(length):
                if 0.5 * bodysizeb/-np.sin(gate_rad) > points_x_plus_y_side[j] > 0.5 * bodysizeb/np.sin(gate_rad)\
                        and points_x_plus_y_bottom[j] > 0:
                    if ranges[j] < safe_margin:
                        detection[i] = [0]
                        break
                    else:
                        detection[i].append(ranges[j])
        elif gate_rad > 0:
            for j in range(length):
                if 0.5 * bodysizeb/-np.sin(gate_rad) < points_x_plus_y_side[j] < 0.5 * bodysizeb/np.sin(gate_rad)\
                        and points_x_plus_y_bottom[j] > 0:
                    if ranges[j] < safe_margin:
                        detection[i] = [0]
                        break
                    else:
                        detection[i].append(ranges[j])
        else:   # gate_rad == 0
            for j in range(length):
                if 0.1 > points[j, 0] > -0.1:
                    if ranges[j] < safe_margin:
                        detection[i] = [0]
                        break
                    else:
                        detection[i].append(ranges[j])
        detection[i] = mean(detection[i])

    max_gate = np.argmax(detection)
    angle_max_range = math.radians(-usable_range_angle/2 + (max_gate + 1) * gate_angle)
    max_range = 10  # only use to visualize the target direction, doesn't matter what value it is.

    p11 = Point(0.5 * bodysizeb/-np.cos(angle_max_range), 0)
    p12 = Point(max_range * -np.sin(angle_max_range) + 0.5 * bodysizeb/-np.cos(angle_max_range),
                max_range * np.cos(angle_max_range))
    p21 = Point(0.5 * bodysizeb / np.cos(angle_max_range), 0)
    p22 = Point(max_range * -np.sin(angle_max_range) + 0.5 * bodysizeb / np.cos(angle_max_range),
                max_range * np.cos(angle_max_range))
    show_line_in_rviz(4, [p11, p12],
                      color=ColorRGBA(1, 0.4, 0, 1), line_width=0.005)
    show_line_in_rviz(5, [p21, p22],
                      color=ColorRGBA(1, 0.4, 0, 1), line_width=0.005)
    show_line_in_rviz(6, [Point(0, 0), Point(0, safe_margin)],
                      color=ColorRGBA(0.5, 0.4, 0.7, 1), line_width=0.02)

    return angle_max_range


def find_left_right_border(points, margin_relative=0.1):
    margin = int(points.shape[0] * margin_relative)

    relative = points[margin + 1:-margin, :] - points[margin:-margin - 1, :]
    distances = np.linalg.norm(relative, axis=1)

    return margin + np.argmax(distances) + 1


def follow_walls(angle_max_range, left_circle, right_circle, barrier, delta_time):
    global last_speed

    prediction_distance = parameters.corner_cutting + \
        parameters.straight_smoothing * last_speed

    predicted_car_position = Point(0, prediction_distance)
    # target_direction = Point(10 * -np.sin(angle_max_range), 10 * np.cos(angle_max_range))
    # left_point = left_circle.get_closest_point(predicted_car_position)
    # right_point = right_circle.get_closest_point(predicted_car_position)

    error = (predicted_car_position.y * - np.tan(angle_max_range)) / \
        prediction_distance

    if math.isnan(error) or math.isinf(error):
        error = 0
    steering_angle = pid.update_and_get_correction(
        error, delta_time)

    radius = min(left_circle.radius, right_circle.radius)

    speed_limit_radius = map(parameters.radius_lower, parameters.radius_upper, 0, 1, radius)  # nopep8
    speed_limit_error = max(0, 1 + parameters.steering_slow_down_dead_zone - abs(error) * parameters.steering_slow_down)  # nopep8
    speed_limit_acceleration = last_speed + parameters.max_acceleration * delta_time
    speed_limit_barrier = map(parameters.barrier_lower_limit, parameters.barrier_upper_limit, 0, 1, barrier) ** parameters.barrier_exponent  # nopep8

    relative_speed = min(
        speed_limit_error,
        speed_limit_radius,
        speed_limit_acceleration,
        speed_limit_barrier
    )
    last_speed = relative_speed
    speed = map(0, 1, parameters.min_throttle, parameters.max_throttle, relative_speed)  # nopep8
    steering_angle = steering_angle * map(parameters.high_speed_steering_limit_dead_zone, 1, 1, parameters.high_speed_steering_limit, relative_speed)  # nopep8
    steering_angle_limit = 0.4

    if abs(steering_angle) > steering_angle_limit:
        steering_angle = steering_angle*steering_angle_limit/abs(steering_angle)
    # print('speed_before:',speed)
    speed = speed * np.cos(abs(steering_angle/(steering_angle_limit*1.1)))
    drive(steering_angle, speed)
    # print('steering_angle:', steering_angle)
    # print('speed:', speed)

    # show_line_in_rviz(2, [left_point, right_point],
    #                   color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    show_line_in_rviz(2, [Point(0, 0), predicted_car_position],
                      color=ColorRGBA(1, 1, 1, 0.3), line_width=0.005)
    # show_line_in_rviz(4, [predicted_car_position,
    #                       target_position], color=ColorRGBA(1, 0.4, 0, 1))

    # show_line_in_rviz(
    #     5, [Point(-2, barrier), Point(2, barrier)], color=ColorRGBA(1, 1, 0, 1))
    #
    # show_line_in_rviz(3, [Point(0, 0), target_direction],
    #                   color=ColorRGBA(1, 0.4, 0, 1), line_width=0.005)


def handle_scan(laser_scan, delta_time):
    if parameters is None:
        return

    points, angle_max_range = get_scan_as_cartesian(laser_scan)

    if points.shape[0] == 0:
        rospy.logwarn("Skipping current laser scan message since it contains no finite values.")  # nopep8
        return

    split = find_left_right_border(points)

    right_wall = points[:split:4, :]
    left_wall = points[split::4, :]

    left_circle = Circle.fit(left_wall)
    right_circle = Circle.fit(right_wall)

    barrier_start = int(points.shape[0] * (0.5 - parameters.barrier_size_realtive))  # nopep8
    barrier_end = int(points.shape[0] * (0.5 + parameters.barrier_size_realtive))  # nopep8
    barrier = np.max(points[barrier_start: barrier_end, 1])

    follow_walls(angle_max_range, left_circle, right_circle, barrier, delta_time)


last_scan = None


def laser_callback(scan_message):
    global last_scan

    scan_time = scan_message.header.stamp.to_sec()
    if last_scan is not None and abs(scan_time - last_scan) > 0.0001 and scan_time > last_scan:  # nopep8
        delta_time = scan_time - last_scan
        handle_scan(scan_message, delta_time)


    last_scan = scan_time


# '''
#     laser scan in simulation:
#
#
#                         (0 degree, middle point)
#                         |
#                         |
#                         |
#                         |
#                         |
#         (90)--------------------------- (-90)
#                         |
#                         |
#                         |
#                         |
# (180 degree, end point) | (-180 degree, start point)
# '''


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
