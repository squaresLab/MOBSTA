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

import abc  # Abstract Base Class
import subprocess


class ReplayInterface(abc.ABCMeta):
    """Primary Interface that defines how replayers can be run by HPSTA

    This Interface should be filled out for every Replayer to be used with
    HPSTA run. 

    Note: Implementing Classes should also be listed in the replay.py 
        valid_replayers dict.
    """

    @abc.abstractmethod
    def launchReplayer(config, stdout=subprocess.STDOUT, stderr=subprocess.STDOUT):
        """Launches a replayer as a subprocess based on a HPSTA run replay config

        Args:
            config (dict): Dict representing json structure for HPSTA run replay config
            stdout (file descriptor): desired stdout for replay process
            stderr (file descriptor): desired stderr for replay process

        Returns:
            subprocess.Popen: Process of the replayer
        """
        pass

    @abc.abstractmethod
    def getLogLength(config):
        """Parses a log file to retrieve the log length for error checking

        Args:
            config (dict): Dict representing json structure for HPSTA run replay config

        Returns:
            logLength: float log length in seconds
        """
        pass
        
