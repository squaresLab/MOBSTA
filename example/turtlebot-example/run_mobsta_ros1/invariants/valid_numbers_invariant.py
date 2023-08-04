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
import invariant_base
import time
import math

class ValidNumbersInvariant(invariant_base.BaseInvariant):
    """
    Defines an invariant that velocities published by the automatic parking node don't contain inf or NaN values
    
    Parameters:
        velocity_topic (str): The topic that the automatic parking node publishes velocities on
        
    Attributes:
        params (dict): dict with parameter info
    """
    
    def __init__(self):
        super(ValidNumbersInvariant, self).__init__()
        self.params = self.get_params()        
        rospy.Subscriber(self.params["velocity_topic"], geometry_msgs.msg.Twist, self.velocityCallback)

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
    ValidNumbersInvariant()
    rospy.spin()


if __name__ == '__main__':
    main()
