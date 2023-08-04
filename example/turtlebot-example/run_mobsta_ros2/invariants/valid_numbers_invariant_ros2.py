#!/usr/bin/python3

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

import rclpy
import geometry_msgs.msg
import invariant_base_ros2
import time
import math

class ValidNumbersInvariantROS2(invariant_base_ros2.BaseInvariantROS2):
    """
    Defines an invariant that velocities published by the automatic parking node don't contain inf or NaN values
    
    Parameters:
        velocity_topic (str): The topic that the automatic parking node publishes velocities on
        
    Attributes:
        params (dict): dict with parameter info
    """
    
    def __init__(self):
        super(ValidNumbersInvariantROS2, self).__init__()
        self.params = self.get_params()        
        self.create_subscription(geometry_msgs.msg.Twist, self.params["velocity_topic"], self.velocityCallback, 10)

    def velocityCallback(self, msg):
        """
        Checks the most recent velocity message for invalid numbers
        
        Args:
            msg (geometry_msgs/Twist): The velocity message being published by the automatic parking node
            
        Returns:
            None
        """
        if math.isnan(msg.linear.x) or math.isnan(msg.angular.z) or math.isinf(msg.linear.x) or math.isinf(msg.angular.z):
             self.report_violation()

        
def main():
    rclpy.init()
    valid_numbers_inv = ValidNumbersInvariantROS2()

    try:
        rclpy.spin(valid_numbers_inv)
    except KeyboardInterrupt:
        pass

    valid_numbers_inv.destroy_node()


if __name__ == '__main__':
    main()
