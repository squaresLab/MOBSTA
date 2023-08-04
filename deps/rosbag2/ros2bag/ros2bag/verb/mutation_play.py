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
import ros2cli
import rosbag2_py
import ros2bag
import yaml
import ros2bag.verb.play
import argparse
import random


class MutationPlayVerb(ros2bag.verb.play.PlayVerb):
    """ros2 bag mutation play."""

    def add_arguments(self, parser, cli_name):
        """adding mutation config file as an extra argument to be passed
        through the command line.

        Args:
            parser (argparser.Parser): parses command line arguments.
            cli_name (str): the name of the verb on the command line.
        
        Returns:
            None
        """
        ros2bag.verb.play.PlayVerb.add_arguments(self, parser, cli_name)
        parser.add_argument('--mutation-config-file', type=str, default='',
            help='JSON file that defines the topics and fields where the mutation are performed')
        parser.add_argument('--seed', type=int, default=random.getrandbits(64),
            help='a seed for deterministic random mutations and chances')


    def main(self, *, args):
        """executes the main thread of the play command and calls the replay library

        Args:
            args (aegparse.Namespace): the command line arguments for the mutation play.
        
        Returns:
            None
        """
        qos_profile_overrides = {}  # Specify a valid default
        if args.qos_profile_overrides_path:
            qos_profile_dict = yaml.safe_load(args.qos_profile_overrides_path)
            try:
                qos_profile_overrides = ros2bag.api.convert_yaml_to_qos_profile(
                    qos_profile_dict)
            except (rclpy.qos.InvalidQoSProfileException, ValueError) as e:
                return ros2bag.api.print_error(str(e))

        storage_config_file = ''
        if args.storage_config_file:
            storage_config_file = args.storage_config_file.name

        topic_remapping = ['--ros-args']
        for remap_rule in args.remap:
            topic_remapping.append('--remap')
            topic_remapping.append(remap_rule)

        storage_options = rosbag2_py.StorageOptions(
            uri=args.bag_path,
            storage_id=args.storage,
            storage_config_uri=storage_config_file,
        )
        play_options = rosbag2_py.PlayOptions()
        play_options.read_ahead_queue_size = args.read_ahead_queue_size
        play_options.node_prefix = ros2cli.node.NODE_NAME_PREFIX
        play_options.rate = args.rate
        play_options.topics_to_filter = args.topics
        play_options.topic_qos_profile_overrides = qos_profile_overrides
        play_options.loop = args.loop
        play_options.topic_remapping_options = topic_remapping
        play_options.clock_publish_frequency = args.clock
        play_options.delay = args.delay
        play_options.disable_keyboard_controls = args.disable_keyboard_controls
        play_options.start_paused = args.start_paused
        play_options.start_offset = args.start_offset
        play_options.wait_acked_timeout = args.wait_for_all_acked
        play_options.disable_loan_message = args.disable_loan_message
        play_options.seed = args.seed

        player = rosbag2_py.MutationPlayer()
        try:
            player.play(storage_options, play_options, args.mutation_config_file)
        except KeyboardInterrupt:
            pass
