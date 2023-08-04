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

from geometry_msgs.msg import Twist
import numpy as np
import math

def determine_next_command(start_point, center_point, end_point, angle_tolerance_deg, position_tolerance, distance_tolerance):
    """    
    if parking spot angle is within angle_tolerance_deg of 0 and the x coordinate of the center point is not within position_tolerance        
        if the spot is behind the robot, drive backward
        if the spot is in front of the robot, drive forward   
    if parking spot angle is within angle_tolerance_deg of 90 degrees and the y coordinate of the center point is within position_tolerance:
        if the robot is within distance_tolerance of the parking spot, stop
        if the spot is behind the robot, drive backward
        if the spot is in front of the robot, drive forward
    otherwise
        draw a line through the robot perpendicular to the parking spot and see where it intersects the line defined by the parking spot
        if the intersection point is within distance_tolerance of the center point, it should command a rotation to point toward the center point
        otherwise, it should line itself up parallel with the parking spot
    """
    cmd_vel = Twist()
    demo_finished = False
    
    theta = np.arctan2(start_point[1] - end_point[1], start_point[0] - end_point[0])
    while (theta) < 0:
        theta = theta + math.pi
    while (theta) > math.pi:
        theta = theta - math.pi
        
    if (theta < np.deg2rad(angle_tolerance_deg) or theta > np.deg2rad(180-angle_tolerance_deg)) and abs(center_point[0]) > position_tolerance:
        if center_point[0] > 0:
            cmd_vel.linear.x = 0.05
        else:
            cmd_vel.linear.x = -0.05

    else:
        # Draw a line through the robot perpendicular to the parking spot and see where it intersects the line defined by the parking spot
        dy = end_point[1] - start_point[1];
        dx = end_point[0] - start_point[0];
        intersection_x = (start_point[0]*dy*dy - start_point[1]*dy*dx)/(dx*dx + dy*dy)
        intersection_y = (start_point[1]*dx*dx - start_point[0]*dy*dx)/(dx*dx + dy*dy)
        
        # Get the distance from that point to the center
        dx = intersection_x - center_point[0]
        dy = intersection_y - center_point[1]
        distance_to_center = math.sqrt(dx*dx + dy*dy)
        
        if distance_to_center > position_tolerance:
            if theta > np.deg2rad(90):
                cmd_vel.angular.z = -0.1
            else:
                cmd_vel.angular.z = 0.1
        else:
            if theta <= np.deg2rad(90 - angle_tolerance_deg):
                cmd_vel.angular.z = -0.1
            elif theta >= np.deg2rad(90 + angle_tolerance_deg):
                cmd_vel.angular.z = 0.1
            else:
                if abs(center_point[0]) < distance_tolerance:
                    demo_finished = True
                else:
                    if center_point[0] > 0:
                        cmd_vel.linear.x = 0.05
                    else:
                        cmd_vel.linear.x = -0.05
                
    return [cmd_vel, demo_finished]
    

    
if __name__=="__main__":
    all_tests_passed = True

    # Run some tests to make sure this does the right thing
        # Should rotate to the right to line up parallel with the parking spot
    [cmd_vel, demo_finished] = determine_next_command([4.897, -0.518], [5.763, -1.018], [6.629, -1.518], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z >= 0:
        all_tests_passed = False
        print('Test 1 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))        

        # Should rotate to the left to line up parallel with the parking spot
    [cmd_vel, demo_finished] = determine_next_command([2.897, 3.982], [3.763, 4.482], [4.629, 4.982], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z <= 0:
        all_tests_passed = False
        print('Test 2 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))        

        # Should rotate to the right to line up parallel with the parking spot
    [cmd_vel, demo_finished] = determine_next_command([-2.897, 3.982], [-2.031, 3.482], [-1.165, 2.982], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z >= 0:
        all_tests_passed = False
        print('Test 3 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))

        # Should rotate to the left to line up parallel with the parking spot
    [cmd_vel, demo_finished] = determine_next_command([-4.897, -0.518], [-4.031, -0.018], [-3.165, 0.482], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z <= 0:
        all_tests_passed = False
        print('Test 4 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))   
        
        
        
        # Should drive forward
    [cmd_vel, demo_finished] = determine_next_command([4.567, 1.842], [5.566, 1.807], [6.566, 1.772], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x <= 0 or cmd_vel.angular.z != 0:
        all_tests_passed = False
        print('Test 5 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))

        # Should drive backward
    [cmd_vel, demo_finished] = determine_next_command([-4.567, -1.842], [-5.566, -1.807], [-6.566, -1.772], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x >= 0 or cmd_vel.angular.z != 0:
        all_tests_passed = False
        print('Test 6 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))



        # Should rotate to the left to line up perpendicular to the parking spot
    [cmd_vel, demo_finished] = determine_next_command([0.143, 2.227], [1.009, 1.727], [1.875, 1.227], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z <= 0:
        all_tests_passed = False
        print('Test 7 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))  

        # Should rotate to the right to line up perpendicular to the parking spot
    [cmd_vel, demo_finished] = determine_next_command([-1.857, 1.237], [-0.991, 1.737], [-0.125, 2.237], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z >= 0:
        all_tests_passed = False
        print('Test 8 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))  
        
        # Should rotate to the right to line up perpendicular to the parking spot
    [cmd_vel, demo_finished] = determine_next_command([1.857, -1.237], [0.991, -1.737], [0.125, -2.237], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z >= 0:
        all_tests_passed = False
        print('Test 9 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))       
        
        # Should rotate to the left to line up perpendicular to the parking spot
    [cmd_vel, demo_finished] = determine_next_command([-1.237, -1.857], [-1.737, -0.991], [-2.237, -0.125], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z <= 0:
        all_tests_passed = False
        print('Test 10 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))            



        # Should drive forward
    [cmd_vel, demo_finished] = determine_next_command([1.982, 1.025], [2.0, 0.025], [2.017, -0.975], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x <= 0 or cmd_vel.angular.z != 0:
        all_tests_passed = False
        print('Test 11 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))

        # Should drive backward
    [cmd_vel, demo_finished] = determine_next_command([-2.017, -0.955], [-1.999, 0.045], [-1.982, 1.044], 5, 0.02, 0.5)
    if demo_finished or cmd_vel.linear.x >= 0 or cmd_vel.angular.z != 0:
        all_tests_passed = False
        print('Test 12 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))



        # Should say demo is finished
    [cmd_vel, demo_finished] = determine_next_command([0.383, 0.997], [0.4, -0.003], [0.418, -1.003], 5, 0.02, 0.5)
    if not demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z != 0:
        all_tests_passed = False
        print('Test 13 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))

        # Should say demo is finished
    [cmd_vel, demo_finished] = determine_next_command([-0.417, -0.983], [-0.4, 0.017], [-0.382, 1.017], 5, 0.02, 0.5)
    if not demo_finished or cmd_vel.linear.x != 0 or cmd_vel.angular.z != 0:
        all_tests_passed = False
        print('Test 14 failed, commanded velocity was {} {}'.format(cmd_vel.linear.x, cmd_vel.angular.z))

        
    if all_tests_passed:
        print('All tests passed')
