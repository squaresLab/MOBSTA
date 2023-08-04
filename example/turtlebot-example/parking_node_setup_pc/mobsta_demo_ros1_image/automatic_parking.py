#!/usr/bin/env python2
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
from math import sin, cos, pi, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

def scan_parking_spot():
    scan_done = False
    angles_in_range = []
    indices_in_range = []
    spot_angle_index = []
    intensity_threshold = 100
    center_angle = 0
    start_angle = 0
    end_angle = 0
    
    for i in range(len(msg.intensities)):
        spot_intensity = msg.intensities[i] ** 2 * msg.ranges[i] / 100
        spot_angle = msg.angle_min + i*msg.angle_increment

        if spot_intensity >= intensity_threshold:
            angles_in_range.append(np.rad2deg(spot_angle))
            indices_in_range.append(i)
        else:
            angles_in_range.append(0)

    for i in indices_in_range:
        if abs(i - indices_in_range[int(len(indices_in_range) / 2)]) < 20:
            spot_angle_index.append(angles_in_range[i])
            if len(spot_angle_index) > 10:
                scan_done = True
                center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
                start_angle = spot_angle_index[2]
                end_angle = spot_angle_index[-3]

            else:
                scan_done = False
    return scan_done, center_angle, start_angle, end_angle, indices_in_range

def quaternion():
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw

def angle_to_index(angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    angle_index = int((angle_rad - msg.angle_min) / msg.angle_increment)
    return angle_index

def get_angle_distance(angle):
    angle_index = angle_to_index(angle)
    distance = msg.ranges[angle_index]
    return angle, distance

def get_point(start_angle_distance):
    angle = np.deg2rad(start_angle_distance[0])
    distance = start_angle_distance[1]

    x = distance * cos(angle)
    y = distance * sin(angle)

    return [x, y]

def finding_spot_position(center_angle, start_angle, end_angle, prev_start_point, prev_center_point, prev_end_point):
    fining_spot = False
    start_angle_distance = get_angle_distance(start_angle)
    center_angle_distance = get_angle_distance(center_angle)
    end_angle_distance = get_angle_distance(end_angle)

    if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
        start_point = get_point(start_angle_distance)
        center_point = get_point(center_angle_distance)
        end_point = get_point(end_angle_distance)
        fining_spot = True
    else:
        start_point = prev_start_point
        center_point = prev_center_point
        end_point = prev_end_point
        fining_spot = False

    return fining_spot, start_point, center_point, end_point


def scan_spot_filter(msg, center_angle, start_angle, end_angle):
    scan_spot_pub = rospy.Publisher("/scan_spot", LaserScan, queue_size=1)
    scan_spot = msg
    scan_spot_list = list(scan_spot.intensities)
    for i in range(len(scan_spot_list)):
        scan_spot_list[i] = 0
        
    start_angle_index = angle_to_index(start_angle)
    center_angle_index = angle_to_index(center_angle)
    end_angle_index = angle_to_index(end_angle)
        
    scan_spot_list[start_angle_index] = msg.ranges[start_angle_index]
    scan_spot_list[center_angle_index] = msg.ranges[center_angle_index]
    scan_spot_list[end_angle_index] = msg.ranges[end_angle_index]
    scan_spot.ranges = tuple(scan_spot_list)
    scan_spot_pub.publish(scan_spot)

if __name__=="__main__":
    rospy.init_node('AutoParking')
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    reset_pub = rospy.Publisher('/reset', Empty, queue_size=1)
    msg = LaserScan()
    r = rospy.Rate(10)
    step = 0
    twist = Twist()
    reset = Empty()
    
    angle_tolerance_deg = 5
    position_tolerance = 0.05
    distance_tolerance = 0.5
    scan_done = False
    center_angle = 0
    start_angle = 0
    end_angle = 0
    indices_in_range = []    
    start_point = [0, 0]    
    center_point = [0, 0]    
    end_point = [0, 0]
    
    reset_pub.publish(Empty())
    time.sleep(3)
    
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("/scan", LaserScan)
        odom = rospy.wait_for_message("/odom", Odometry)
        yaw = quaternion()
        
        scan_done, center_angle, start_angle, end_angle, indices_in_range = scan_parking_spot()
        
        if scan_done:
            fining_spot, start_point, center_point, end_point = finding_spot_position(center_angle, start_angle, end_angle, start_point, center_point, end_point)
            theta = np.arctan2(start_point[1] - end_point[1], start_point[0] - end_point[0])
            while (theta) < 0:
                theta = theta + pi
            while (theta) > pi:
                theta = theta - pi

        if step == 0:
            if scan_done:
                if fining_spot:
                    print("=================================")
                    print("|        |     x     |     y     |")
                    print('| start  | {0:>10.3f}| {1:>10.3f}|'.format(start_point[0], start_point[1]))
                    print('| center | {0:>10.3f}| {1:>10.3f}|'.format(center_point[0], center_point[1]))
                    print('| end    | {0:>10.3f}| {1:>10.3f}|'.format(end_point[0], end_point[1]))
                    print("=================================")
                    print('| theta  | {0:.2f} deg'.format(np.rad2deg(theta)))
                    print('| yaw    | {0:.2f} deg'.format(np.rad2deg(yaw)))
                    print("=================================")
                    print("===== Rotate to parking spot!!! =====")
                    step = 1
            else:
                print("Fail finding parking spot.")

        elif step == 1:
            if theta < np.deg2rad(angle_tolerance_deg) or theta > np.deg2rad(180-angle_tolerance_deg):
                print("===== Move to parking spot!!!! =====")
                twist.angular.z = 0.0
                cmd_pub.publish(twist)
                step = 2                            
            elif theta < pi/2.0:
                twist.angular.z = 0.1
            else:
                twist.angular.z = -0.1

        elif step == 2:
            if center_point[0] < -position_tolerance:
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            elif center_point[0] > position_tolerance:
                twist.linear.x = 0.05
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("===== Rotate to line up!!!!! =====")
                step = 3

        elif step == 3:
            if theta > np.deg2rad(90 - angle_tolerance_deg) and theta < np.deg2rad(90 + angle_tolerance_deg):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("===== Get closer!!!!!! =====")
                cmd_pub.publish(twist)
                step = 4
            elif theta <= np.deg2rad(90 - angle_tolerance_deg):
                twist.linear.x = 0.0
                twist.angular.z = -0.1
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.1

        elif step == 4:
            if center_point[0] < -distance_tolerance:
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            elif center_point[0] > distance_tolerance:
                twist.linear.x = 0.05
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("Auto_parking Done!!!!!!!")
                cmd_pub.publish(twist)
                step = -1
        cmd_pub.publish(twist)
        
        if scan_done:
            scan_spot_filter(msg, center_angle, start_angle, end_angle)
