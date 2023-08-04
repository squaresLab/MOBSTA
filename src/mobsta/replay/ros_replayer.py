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
import os
import subprocess
import tempfile
import json
import rosbag
import yaml


class RosReplayer(replay_interface.ReplayInterface):
    """Replay api used by HPSTA run to launch a Ros log Replay.

    Config Parameters:
        replay-log (str): path to rosbag to replay
        replay-delay (float): float seconds to wait before starting the bag
        mutation-config (dict): json-like dict to be passed to 
            mutational replay as the mutation config
        seed (int): seed to provide for deterministic replay 
        cmd-line-args (str): Optional. Additional command line args to 
            pass to rosbag replay


    Attributes:
        MUTATIONAL_REPLAY (str): filepath to mutational replay executable
        REQUIRED_KEYS (list[str]): list of keys that must be in the replay config

    """

    MUTATIONAL_REPLAY = os.path.abspath(os.path.join(
                            os.path.abspath(__file__),
                            "../../../../deps/ros_comm/tools/rosbag/build/devel/lib/rosbag/play")
                        )
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
        for key in RosReplayer.REQUIRED_KEYS:
            if key not in config:
                raise KeyError("Required Key {} not in Replay Config".format(key))

        log_file = os.path.expandvars(config["replay-log"])
        if not os.path.exists(log_file):
            raise OSError("Log File not found: {}".format(log_file))

        # Launch process
        with tempfile.NamedTemporaryFile(mode='w', delete=False) as mutation_config_file:
            json.dump(config["mutation-config"], mutation_config_file)

            replay_command = [RosReplayer.MUTATIONAL_REPLAY,
                              "--config-file", mutation_config_file.name,
                              "-d", str(config["replay-delay"]),
                              "--seed", str(config["seed"]),
                              str(os.path.expandvars(config["replay-log"]))
                              ]

            if "cmd-line-args" in config:
                replay_command.append(config["cmd-line-args"])

            return subprocess.Popen(replay_command, stdout=stdout, stderr=stderr, preexec_fn=os.setsid)

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
        info_dict = yaml.load(rosbag.Bag(log_file, 'r')._get_yaml_info())
        return info_dict["duration"]
