#!/usr/bin/env python3

"""
@ 2023 Carnegie Mellon University. All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University
www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written
permission.

License Status: Not Released.
(License Status to be confirmed by CTTEC prior to release from NREC)
This notice must appear in all copies of this file and its derivatives.
"""

"""
NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project
Manager/PI prior to use on another project).
Created for Program: HPSTA - 55435.1.1990813
"""

import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import invariant_base
import parking_utils
import numpy
import odom_from_joint_states
import time

class CorrectCommandsInvariant(invariant_base.BaseInvariant):
    """
    Defines an invariant that the parking demo executes the "proper" command based on the ground truth of where
    the parking spot is. Basically, checks that the robot is following the steps described in the readme
    
    Parameters:
        velocity_topic (str): The topic that the automatic parking node rerun will publish velocities on
        joint_states_topic (str): The topic that contains the logged joint states, used for computing odometry
        
        bad_command_limit (int): The number of "wrong" commands we need to get before triggering a violation
        
        parking_spot_x_1, parking_spot_x_2 (float): x coordinates of the two points that define the line of where the parking spot is
        parking_spot_y_1, parking_spot_y_2 (float): y coordinates of the two points that define the line of where the parking spot is
        
        angle_goal_deg (float): When rotating to line up with the parking spot, how close to 90 or 0 degrees the robot should be before moving
        position_goal (float): When driving to line up the y axis with the parking spot, how close it needs to be before rotating again
        distance_goal (float): When driving toward the parking spot, how close the robot needs to be before stopping

        angle_uncertainty_deg (float): A measure of how far off we think the odometry angle estimate might be
        position_uncertainty (float): A measure of how far off we thing the odometry position estimate might be
        
    Attributes:
        odometry (2D pose): Object that estimates odometry based on joint states-- needed because the logged odometry only has x
        parking_spot (array of floats): two points that define the line segment where the parking spot is
        center_point (array of floats): the center of the parking_spot line segment
    """
    
    def __init__(self):
        super(CorrectCommandsInvariant, self).__init__()
        self.params = self.get_params()
              
        rospy.Subscriber(self.params["velocity_topic"], geometry_msgs.msg.Twist, self.velocityCallback)
        rospy.Subscriber(self.params["joint_states_topic"], sensor_msgs.msg.JointState, self.jointStatesCallback)
        self.angle_goal_deg = self.params["angle_goal_deg"]
        self.position_goal = self.params["position_goal"]
        self.distance_goal = self.params["distance_goal"]
        self.bad_command_limit = self.params["bad_command_limit"]
        self.angle_uncertainty = self.params["angle_uncertainty_deg"]
        self.position_uncertainty = self.params["position_uncertainty"]
        
        self.bad_command_count = 0

        # Object that estimates odometry from the joint states
        self.odometry = odom_from_joint_states.OdomFromJointStates()

        # Store the parking spot as an array of three vectors so that it's easy to transform just
        # with a matrix multiplication
        x_start = self.params["parking_spot_x_1"]
        x_end = self.params["parking_spot_x_2"]
        x_center = 0.5*(x_start+x_end)
        y_start = self.params["parking_spot_y_1"]
        y_end = self.params["parking_spot_y_2"]
        y_center = 0.5*(y_start+y_end)

        self.parking_spot = numpy.array([[x_start, x_center, x_end], [y_start, y_center, y_end], [1, 1, 1]])
        
    def transformToLocalFrame(self, pose_vector):
        local_frame_parking_spot = odom_from_joint_states.poseVectorToMatrix(pose_vector).getI() * self.parking_spot
        local_frame_start_point = [local_frame_parking_spot[0,0], local_frame_parking_spot[1,0]]
        local_frame_center_point = [local_frame_parking_spot[0,1], local_frame_parking_spot[1,1]]
        local_frame_end_point = [local_frame_parking_spot[0,2], local_frame_parking_spot[1,2]]

        return [local_frame_start_point, local_frame_center_point, local_frame_end_point]
        
    def velocityCallback(self, msg):
        callback_start_time = time.time()
        pose_vector = self.odometry.getPoseAsVector()

        command_passed = False

        # Figure out where the parking spot should be based on the pose
        for x_offset in numpy.arange(-self.position_uncertainty, self.position_uncertainty, 0.1):
            for y_offset in numpy.arange(-self.position_uncertainty, self.position_uncertainty, 0.1):
                for yaw_offset in numpy.deg2rad(numpy.arange(-self.angle_uncertainty, self.angle_uncertainty, 1.0)):
                    [local_frame_start_point, local_frame_center_point, local_frame_end_point] = self.transformToLocalFrame(pose_vector + numpy.array([x_offset, y_offset, yaw_offset]))

                    # Determine what this command should be
                    [next_velocity_command, _] = parking_utils.determine_next_command(local_frame_start_point, local_frame_center_point, local_frame_end_point, self.angle_goal_deg, self.position_goal, self.distance_goal)

                    # Does it match?
                    if abs(next_velocity_command.linear.x-msg.linear.x) < 0.001 and abs(next_velocity_command.angular.z-msg.angular.z) < 0.001:
                        self.bad_command_count = 0
                        command_passed = True

        if not command_passed:
            self.bad_command_count = self.bad_command_count + 1
            if self.bad_command_count > self.bad_command_limit:
                self.report_violation()

        callback_end_time = time.time()

            
    def jointStatesCallback(self, msg):
        self.odometry.updatePose(msg)
        pose_vector = self.odometry.getPoseAsVector()

def main():
    CorrectCommandsInvariant()
    rospy.spin()

if __name__ == '__main__':
    main()
        
