#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Gilbert, Ryan Shim

import math
import numpy
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


class Turtlebot3AutomaticParking(Node):
    def __init__(self):
        super().__init__('turtlebot3_automatic_parking')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 0
        self.scan = []
        self.rotation_point = []
        self.center_point = []
        self.theta = 0.0
        self.yaw = 0.0
        self.get_key_state = False
        self.init_scan_state = False  # To get the initial scan at the beginning
        self.init_odom_state = False  # To get the initial odom at the beginning
        
        self.angle_tolerance_deg = 5
        self.position_tolerance = 0.05
        self.distance_tolerance = 0.5

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.reset_pub = self.create_publisher(Empty, 'reset', qos)
        self.scan_spot_pub = self.create_publisher(LaserScan, 'scan_spot', qos)
                
        self.reset_pub.publish(Empty())
        time.sleep(3)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 automatic parking node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scan_callback(self, msg):
        self.scan = msg
        self.init_scan_state = True

    def odom_callback(self, msg):
        self.odom = msg
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.yaw = self.last_pose_theta
        self.init_odom_state = True

    def update_callback(self):
        if self.init_scan_state is True and self.init_odom_state is True:
            self.park_robot()

    def park_robot(self):
        scan_done, center_angle, start_angle, end_angle, indices_in_range = self.scan_parking_spot()
        twist = Twist()
        reset = Empty()
        fining_spot = False
        
        if scan_done:
            fining_spot, start_point, self.center_point, end_point = self.find_parking_spot(center_angle, start_angle, end_angle)
            self.theta = numpy.arctan2(start_point[1] - end_point[1], start_point[0] - end_point[0])
            while (self.theta) < 0:
                self.theta = self.theta + math.pi
            while (self.theta) > math.pi:
                self.theta = self.theta - math.pi

        # Step 0: Find a parking spot
        if self.step == 0:
            if scan_done:
                if fining_spot:
                    print("=================================")
                    print("|        |     x     |     y     |")
                    print('| start  | {0:>10.3f}| {1:>10.3f}|'.format(start_point[0], start_point[1]))
                    print('| center | {0:>10.3f}| {1:>10.3f}|'.format(self.center_point[0], self.center_point[1]))
                    print('| end    | {0:>10.3f}| {1:>10.3f}|'.format(end_point[0], end_point[1]))
                    print("=================================")
                    print('| theta  | {0:.2f} deg'.format(numpy.rad2deg(self.theta)))
                    print('| yaw    | {0:.2f} deg'.format(numpy.rad2deg(self.yaw)))
                    print("=================================")
                    print("===== Rotate to parking spot!!! =====")
                    self.step = 1
            else:
                print("Fail finding parking spot.")

        # Step 1: Turn
        elif self.step == 1:
            if self.theta < numpy.deg2rad(self.angle_tolerance_deg) or self.theta > numpy.deg2rad(180-self.angle_tolerance_deg):
                print("===== Move to parking spot!!!! =====")
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.step = 2                            
            elif self.theta < math.pi/2.0:
                twist.angular.z = 0.1
            else:
                twist.angular.z = -0.1

        # Step 2: Move straight
        elif self.step == 2:
            if self.center_point[0] < -self.position_tolerance:
                twist.linear.x = -0.05
                twist.angular.z = 0.0
            elif self.center_point[0] > self.position_tolerance:
                twist.linear.x = 0.05
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("===== Rotate to line up!!!!! =====")
                self.step = 3

        # Step 3: Turn
        elif self.step == 3:
            if self.theta > numpy.deg2rad(90 - self.angle_tolerance_deg) and self.theta < numpy.deg2rad(90 + self.angle_tolerance_deg):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("===== Get closer!!!!!! =====")
                self.step = 4
            elif self.theta <= numpy.deg2rad(90 - self.angle_tolerance_deg):
                twist.linear.x = 0.0
                twist.angular.z = -0.1
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.1

        # Step 4: Move Straight
        elif self.step == 4:
            ranges = []
            for i in indices_in_range:
                if self.scan.ranges[i] != 0:
                    ranges.append(self.scan.ranges[i])
            twist.linear.x = 0.0                    
            twist.angular.z = 0.0
            
            if len(ranges) > 0:
                if min(ranges) > self.distance_tolerance:
                    if self.center_point[0] > 0:
                        twist.linear.x = 0.05
                    else:
                        twist.linear.x = -0.05
                else:
                    print("Automatic parking done.")
                    self.cmd_vel_pub.publish(twist)
                    self.step = -1
                
        self.cmd_vel_pub.publish(twist)

        if scan_done:
            self.scan_spot_filter(center_angle, start_angle, end_angle)

    def scan_parking_spot(self):
        scan_done = False
        indices_in_range = []
        spot_angle_index = []
        intensity_threshold = 230
        center_angle = 0
        start_angle = 0
        end_angle = 0
    
        for i in range(len(self.scan.intensities)):
            spot_intensity = self.scan.intensities[i]
            spot_angle = self.scan.angle_min + i*self.scan.angle_increment

            if spot_intensity >= intensity_threshold:
                indices_in_range.append(i)
                spot_angle_index.append(numpy.rad2deg(spot_angle))
            elif not scan_done:
                indices_in_range.clear()
                spot_angle_index.clear()
            else:
                break
        
            if len(spot_angle_index) > 6:
                scan_done = True
                
        if scan_done:
            center_angle = spot_angle_index[int(len(spot_angle_index) / 2)]
            start_angle = spot_angle_index[2]
            end_angle = spot_angle_index[-3]
            
        return scan_done, center_angle, start_angle, end_angle, indices_in_range

    def angle_to_index(self, angle_deg):
        angle_rad = numpy.deg2rad(angle_deg)
        angle_index = int((angle_rad - self.scan.angle_min) / self.scan.angle_increment)
        return angle_index
    
    def get_angle_distance(self, angle):
        angle_index = self.angle_to_index(angle)
        distance = self.scan.ranges[angle_index]

        if math.isnan(distance):
            distance = 0

        return angle, distance

    def get_point(self, start_angle_distance):
        angle = start_angle_distance[0]
        angle = numpy.deg2rad(angle)
        distance = start_angle_distance[1]

        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        return [x, y]

    def find_parking_spot(self, center_angle, start_angle, end_angle):
        fining_spot = False
        start_angle_distance = self.get_angle_distance(start_angle)
        center_angle_distance = self.get_angle_distance(center_angle)
        end_angle_distance = self.get_angle_distance(end_angle)

        start_point = [0, 0]
        center_point = [0, 0]
        end_point = [0, 0]

        if start_angle_distance[1] != 0 and center_angle_distance[1] != 0 and end_angle_distance[1] != 0:
            start_point = self.get_point(start_angle_distance)
            center_point = self.get_point(center_angle_distance)
            end_point = self.get_point(end_angle_distance)
            fining_spot = True
        else:
            fining_spot = False

        return fining_spot, start_point, center_point, end_point

    def rotate_origin_only(self, x, y, radians):
        xx = x * math.cos(radians) + y * math.sin(radians)
        yy = -x * math.sin(radians) + y * math.cos(radians)
        return xx, yy

    def scan_spot_filter(self, center_angle, start_angle, end_angle):
        scan_spot = self.scan
        scan_spot_list = list(scan_spot.ranges)
        for i in range(len(scan_spot_list)):
            scan_spot_list[i] = 0.0

        start_angle_index = self.angle_to_index(start_angle)
        center_angle_index = self.angle_to_index(center_angle)
        end_angle_index = self.angle_to_index(end_angle)                        
            
        scan_spot_list[start_angle_index] = self.scan.ranges[start_angle_index]
        scan_spot_list[center_angle_index] = self.scan.ranges[center_angle_index]
        scan_spot_list[end_angle_index] = self.scan.ranges[end_angle_index]
        scan_spot.ranges = tuple(scan_spot_list)
        self.scan_spot_pub.publish(scan_spot)
        
        
    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

