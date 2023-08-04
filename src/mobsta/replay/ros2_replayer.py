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

from . import replay_interface
import subprocess
import os
import json
import tempfile
import yaml


class Ros2Replayer(replay_interface.ReplayInterface):
    MUTATIONAL_REPLAY_PATH = os.path.abspath(os.path.join(
                            os.path.abspath(__file__),
                            "../../../../deps/rosbag2"))

    REQUIRED_KEYS = ["replay-log",
                     "replay-delay",
                     "mutation-config",
                     "seed"
                     ]

    @staticmethod
    def launchReplayer(config, stdout=subprocess.STDOUT, stderr=subprocess.STDOUT):
        """Launches a replayer as a subprocess based on a HPSTA run replay config

        Args:
            config (dict): Dict representing json structure for HPSTA run replay config
            stdout (file descriptor): desired stdout for replay process
            stderr (file descriptor): desired stderr for replay process

        Returns:
            subprocess.Popen: Process of the replayer

        Raises:
            KeyError: Raised if a required key is not in the config
            OSError: Raised if the Bag file does not exist    
        """
        # Input Checking
        for key in Ros2Replayer.REQUIRED_KEYS:
            if key not in config:
                raise KeyError("Required Key {} not in Replay Config".format(key))

        log_path = os.path.expandvars(config["replay-log"])
        if not os.path.exists(log_path):
            raise OSError("Log File not found: {}".format(log_path))


        with tempfile.NamedTemporaryFile(mode='w', delete=False) as mutation_config_file:
            json.dump(config["mutation-config"], mutation_config_file)
            
            replay_command = [os.path.join(Ros2Replayer.MUTATIONAL_REPLAY_PATH, "run_ros2_bag.sh"), log_path, mutation_config_file.name]
            
            return subprocess.Popen(replay_command, stderr=stderr, preexec_fn=os.setsid)

    @staticmethod
    def getLogLength(config):
        """Parses a log file to retrieve the log length for error checking

        Args:
            config (dict): Dict representing json structure for HPSTA run replay config

        Returns:
            logLength: float log length in seconds

        Raises:
            KeyError: Raised if a required key is not in the config
            OSError: Raised if the Bag file does not exist    
        """
        # Input Checking
        if "replay-log" not in config:
            raise KeyError("Required Key {} not in Replay Config".format("replay-log"))

        log_file = os.path.expandvars(config["replay-log"])
        if not os.path.exists(log_file):
            raise OSError("Log File not found: {}".format(log_file))

        # Parse Bag Data 
        metadata_file_path = os.path.join(log_file, 'metadata.yaml')
        with open(metadata_file_path, 'r') as metadata_file:
            info_dict = yaml.safe_load(metadata_file)
            duration_in_nanosecs = info_dict['rosbag2_bagfile_information']['duration']['nanoseconds']
        duration_in_secs = duration_in_nanosecs / 1000000000.0
        return duration_in_secs
