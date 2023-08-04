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

from . import ros_replayer

valid_replayers = {
    "ROS": ros_replayer.RosReplayer,
}


def get_replayer(replayer_key):
    """Returns the replayer Object associated with the replay key in 
    the valid_replayers list.

    Args:
        replayer_key (str): String used as the key for the replayer 
            in the replay.valid_replayers list

    Returns:
        ReplayInterface: Replayer specific implementation of the
            replay.replay_interface.ReplayInterface

    Raises:
        KeyError: If replayer_key not found in valid_replayers
    """
    if replayer_key not in valid_replayers:
        raise KeyError("Key '{}' not found in replay.valid_replayers".format(replayer_key))

    return valid_replayers[replayer_key]
