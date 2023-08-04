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
import math
import numpy

def poseVectorToMatrix(pose_vector):
    pose_matrix = numpy.matrix(numpy.identity(3))
    
    pose_matrix[0,0] = math.cos(pose_vector[2])
    pose_matrix[0,1] = -math.sin(pose_vector[2])
    pose_matrix[0,2] = pose_vector[0]

    pose_matrix[1,0] = math.sin(pose_vector[2])
    pose_matrix[1,1] = math.cos(pose_vector[2])
    pose_matrix[1,2] = pose_vector[1]

    return pose_matrix


class OdomFromJointStates:
    """
    This class exists because there's something wonky with the default odometry that runs on the
    OpenCR board, so we need to compute it ourselves. I think the most reliable topic for this
    would be the joint states message, since it contains the encoder positions of each of the
    wheels
    """

    def __init__(self):
        # Set the initial pose at zero
        self.pose_estimate = [0, 0, 0]
        self.pose_matrix = numpy.matrix(numpy.identity(3))

        # Have not received a joint state message yet
        self.got_first_joint_positions = False
        self.last_joint_positions = [0, 0]

        # Configuration
        self.ticks_per_meter = 30.0
        self.robot_width = 0.165

    def updatePose(self, joint_states_message):
        # Zeros means we got a reset
        if joint_states_message.position[0] == 0 and joint_states_message.position[1] == 0:
            self.got_first_joint_positions = False

        if self.got_first_joint_positions:
            left_diff = (joint_states_message.position[0] - self.last_joint_positions[0])/self.ticks_per_meter
            right_diff = (joint_states_message.position[1] - self.last_joint_positions[1])/self.ticks_per_meter
            dx = 0.5*(left_diff + right_diff)
            dyaw = (right_diff-left_diff)/self.robot_width

            self.pose_estimate[0] = self.pose_estimate[0] + dx*math.cos(self.pose_estimate[2])
            self.pose_estimate[1] = self.pose_estimate[1] + dx*math.sin(self.pose_estimate[2])
            self.pose_estimate[2] = self.pose_estimate[2] + dyaw

            self.pose_matrix = poseVectorToMatrix(self.pose_estimate)

        self.got_first_joint_positions = True
        self.last_joint_positions = joint_states_message.position

    def getPoseAsVector(self):
        return numpy.array(self.pose_estimate)

    def getPoseAsMatrix(self):
        return self.pose_matrix