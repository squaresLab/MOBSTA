"""
@ 2022 Carnegie Mellon University. All rights reserved.
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
import time
import argparse
import json
from diagnostic_msgs.msg import KeyValue


class BaseInvariant(object):
    """Base invariant 

    Args:
        name (str): defines the Name of the Invariant Node for violation 
            reporting purposes
        param_file (filename): path to file with invariant parameters json

    Attributes:
        VIOLATIONS_TOPIC (str): The topic to publish violations on
        name (str): The name of this invariant instance

    """

    VIOLATIONS_TOPIC = "/violations"

    def __init__(self):  # noqa: D107
        self._start_time = time.time()
        self.read_args()

        rospy.init_node(self.name)

        self._pub = rospy.Publisher(BaseInvariant.VIOLATIONS_TOPIC, KeyValue, queue_size=10)

    def read_args(self):
        """Reads in the command line arguments for an invariant
        
        Args:
            name (str): name used for the invariant in reporting
            param_file (dict): dict of invariant_parameters

        """
        parser = argparse.ArgumentParser()
        parser.add_argument('node_name')
        parser.add_argument('--param-file',
                            '-p',
                            help='json file containing the parameters for this invariant',
                            default=None)
        args = parser.parse_args()

        self.name = args.node_name
        self.param_file = args.param_file

    def get_params(self):
        """returns param dict
        
        returns:
            dict: json od invariant parameters loaded into a dict

        raises:
            IOError: if no parameter file provided
        """
        if self.param_file is not None:
            with open(self.param_file, 'r') as fc:
                param_str = fc.read()
            return json.loads(param_str)
        else:
            raise IOError("Parameter file not provided")

    def report_violation(self):
        """Publish a timestamped violation on the VIOLATIONS_TOPIC channel."""
        duration = time.time() - self._start_time
        violation = KeyValue(self.name, str(duration))

        self._pub.publish(violation)
