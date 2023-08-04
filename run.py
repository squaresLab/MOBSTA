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
Created for Program: MOBSTA - 55435.1.1990813
"""


import tempfile
import time
import datetime
import subprocess
import argparse
import json
import os
import sys
import signal
import random
import warnings


MOBSTA_SRC_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'src')
sys.path.append(MOBSTA_SRC_PATH)

from mobsta.monitor import monitor
from mobsta import replay


class MOBSTA():
    """The main scaffold for MOBSTA.

    Runs the SUT, Monitor and Log Replay as child processes.

    Attributes:
        LOG_REPLAY(str): The name for the log replay process
        SUT(str): The name for the System Under Test (SUT) process
        MONITOR(str): The name for the monitor process
        SLEEP_TIME(float): The sleep time inside the loop for 
            the main thread while monitoring the other processes. 
            Doesn't really matter how long this is, so this value was chosen
            as it was aesthetically pleasing

        test_dir(str): The path to the directory containing testing resources
        harness_script(str): Path to the script that will lanch the SUT
        replay_config(str): Path to the configuration file for the log replay tool
        monitor_config(str): Path to the configuration file for the monitor

        log_directory(str): The path to write debug logs to
        processes(dict[str,POpen]): A map of the running subprocesses
        debug_log_files(dict[str,file]): A map from process name to the 
            writable file object for that process
    """

    LOG_REPLAY = 'Log_Replay'
    SUT = 'SUT'
    MONITOR = 'MONITOR'
    SLEEP_TIME = 0.307
 
    def __init__(self, args):
        """Constructor
        
        Args:
            args(argparse.Namespace): The command line args for this test
        """
        self.test_dir = os.path.abspath(args.test_dir)
        self.harness_script = os.path.join(self.test_dir, args.harness_script)
        self.replay_config = os.path.join(self.test_dir, args.replay_config)
        self.monitor_config = os.path.join(self.test_dir, args.monitor_config)
        
        self.log_directory = args.log_directory
        self.processes = {}
        self.debug_log_files = {}

        self.seed = args.seed

    def fail_fmt(self, failed_process_name):
        '''Print an error message for a failed component

        Args:
            failed_process_name (str): provides the name of the processes

        Returns:
            None
        '''
        print('---------------------')
        print('{} Crashed'.format(failed_process_name))
        if failed_process_name in self.debug_log_files:
            debug_log_file_name = self.debug_log_files[failed_process_name].name
            print('Check {} for more details'.format(debug_log_file_name))
        print('---------------------')

    def clean_up(self):
        '''Ensures that all the processes and files are closed before terminting the Program

        Returns:
            None
        '''
        for p_name in self.processes:
            p = self.processes[p_name]
            if p is not None:
                p.poll()
                if p.returncode is None:
                    # Kill the whole group to cover SUTs that don't handle
                    # child processes well
                    os.killpg(p.pid, signal.SIGTERM)

                _, p_stderr = p.communicate()
                # Always log the stderr from every process
                self.debug_log_files[p_name].write(p_stderr.decode())

        for f in self.debug_log_files.values():
            f.close()

    def get_replay_log_length(self, replay_config_json):
        """Access the log replay API to get the log length

        Args:
            replay_config_json(dict[str,obj]): The JSON object containing the 
                configuration for the log replay

        Returns:
            float log length in seconds
        """
        debug_log_file = self.debug_log_files[MOBSTA.LOG_REPLAY]        

        try:
            replay_type = replay_config_json["replay-type"]
            replay_interface = replay.get_replayer(replay_type)

            return replay_interface.getLogLength(replay_config_json)

        except (OSError, KeyError) as e:
            self.fail_fmt(MOBSTA.LOG_REPLAY)
            err_str = "Log Replayer Error: {}\n".format(str(e))
            debug_log_file.write(err_str)
            self.clean_up()
            raise RuntimeError(err_str)

    def launch_SUT(self):
        """Launch the SUT.

        Raises: 
            OSError: Raised if the process can't be launched
        """
        debug_log_file = self.debug_log_files[MOBSTA.SUT]
        try:
            self.processes[MOBSTA.SUT] = subprocess.Popen(
                [self.harness_script],
                stdout=debug_log_file,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid)
        except OSError as e:
            self.fail_fmt(MOBSTA.SUT)
            debug_log_file.write(str(e))
            self.clean_up()
            raise e

    def launch_monitor(self, metadata_file, monitor_delay=0.0):
        """Launch the monitor.

        Params:
            metadata_file (file): file with json metadata to be appended to monitor report
            monitor_delay (float): seconds to delay monitor launch in case SUT restarts roscore

        Raises: 
            OSError: Raised if the process can't be launched
        """
        monitor_file = monitor.callable_path()
        debug_log_file = self.debug_log_files[MOBSTA.MONITOR]
        try:
            self.processes[MOBSTA.MONITOR] = subprocess.Popen(
                [
                    monitor_file,
                    self.monitor_config,
                    '--test-dir',
                    self.test_dir,
                    '--metadata-file',
                    metadata_file,
                    '--delay',
                    str(monitor_delay)
                ],
                stdout=debug_log_file,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid)
        except OSError as e:
            self.fail_fmt(MOBSTA.MONITOR)
            debug_log_file.write(str(e))
            self.clean_up()
            raise e
    
    def launch_replay(self, replay_config_json):
        """Launch the log replayer.

        Args:
            replay_config_json(dict[str,obj]): The JSON object containing the 
                configuration for the log replay

        Raises: 
            OSError: Raised if the process can't be launched
        """
        debug_log_file = self.debug_log_files[MOBSTA.LOG_REPLAY]        

        try:
            replay_type = replay_config_json["replay-type"]
            replay_interface = replay.get_replayer(replay_type)

            self.processes[MOBSTA.LOG_REPLAY] = replay_interface.launchReplayer(
                    replay_config_json, 
                    stdout=debug_log_file, 
                    stderr=subprocess.PIPE
                )

        except (OSError, KeyError) as e:
            self.fail_fmt(MOBSTA.LOG_REPLAY)
            err_str = "Log Replayer Error: {}\n".format(str(e))
            debug_log_file.write(err_str)
            self.clean_up()
            raise OSError(err_str)

    def create_debug_log_files(self):
        """Create timestamped log directory and logfiles."""
        curr_time_str = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
        log_dir = os.path.join(self.test_dir, self.log_directory, curr_time_str)

        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        self.debug_log_files.update({
            MOBSTA.SUT : open(os.path.join(log_dir, MOBSTA.SUT + ".log"), 'w'),
            MOBSTA.MONITOR : open(os.path.join(log_dir, MOBSTA.MONITOR + ".log"), 'w'),
            MOBSTA.LOG_REPLAY : open(os.path.join(log_dir, MOBSTA.LOG_REPLAY + ".log"), 'w') })

    def run_test(self):
        """Run a MOBSTA test.""" 
        start_time = time.time()

        def spinner(): 
            SPIN = "|/-\\"
            while True:
                yield SPIN[0]
                SPIN = SPIN[1:] + SPIN[0]

        _spin = spinner()

        self.create_debug_log_files()

        # Parse the replay config
        with open(self.replay_config, 'r') as fl:
            replay_config_json = json.loads(fl.read())
        test_time = replay_config_json['test-time']
        replay_delay = replay_config_json["replay-delay"]

        monitor_delay = replay_config_json.get("monitor-delay", 0)

        # Error if two seeds are provided
        if "seed" in replay_config_json and self.seed is not None:
            self.fail_fmt("MOBSTA")
            self.clean_up()
            raise RuntimeError("Two Seeds provided from Command line and Replay Config." +
                               "Unable to determine correct seed source.")

        if "seed" not in replay_config_json:
            if self.seed is not None:
                replay_config_json["seed"] = self.seed
            else:
                replay_config_json["seed"] = random.getrandbits(32)
            with tempfile.NamedTemporaryFile(mode='w', delete=False) as replay_metadata_file:
                json.dump(replay_config_json, replay_metadata_file) 
                self.replay_config = replay_metadata_file.name

        log_length = self.get_replay_log_length(replay_config_json)

        if (replay_delay + log_length) > test_time:
            warningMsg = ("MOBSTA: "
                          "replay-delay {replay_delay} + Log Length {log_length}"
                          " is greater than test-time {test_time}.\n".format(
                            replay_delay=replay_delay, test_time=test_time, log_length=log_length)
                          )
            warnings.warn(warningMsg)

        # Launch the SUT and monitor
        self.launch_SUT()
        self.launch_monitor(self.replay_config, monitor_delay=monitor_delay)

        # Launch the replay
        self.launch_replay(replay_config_json)

        # Main test loop: 
        #   Track the health of the various processes
        duration = time.time() - start_time
        try:
            running = True
            while running and duration < test_time:
                # check for running violations
                if MOBSTA.LOG_REPLAY in self.processes and MOBSTA.SUT not in self.processes:
                    self.fail_fmt('MOBSTA')
                    # Since we don't know the specific log, grab the dir from one of the paths
                    log_dir = os.path.basename(self.debug_log_files.values()[0].name)
                    raise RuntimeError('The log is still playing even though the SUT '
                                    + 'has exited normally.\n  Check test '
                                    + 'configuration. Review the log files in '
                                    + '{} for more details.\n'.format(log_dir)
                                    )

                if MOBSTA.MONITOR not in self.processes:
                    self.fail_fmt('MOBSTA')
                    raise RuntimeError('Monitor ended unexpectedly \n'
                                    + 'Monitor process closed while '
                                    + 'test still running.\n')

                # update process status
                _processes = dict(self.processes)
                for name, p in _processes.items():
                    p.poll()
                    ret = p.returncode

                    # Process exited
                    if ret is not None:
                        # Always log the stderr from every process
                        p_stderr = p.stderr.read()
                        self.debug_log_files[name].write(p_stderr.decode())

                        # Process crashed
                        if ret != 0:
                            err_string = p_stderr.decode()
                            self.fail_fmt(name)
                            running = False

                            # The SIGINT propagates from the monitor to the Crash
                            # Invaraint to report that a crash happened.
                            if name == MOBSTA.SUT:
                                # SUT is expected to crash sometimes, MOBSTA expects this
                                self.processes[MOBSTA.MONITOR].send_signal(signal.SIGINT)
                                # Print the SUT error, for clarity, but don't raise because
                                #   the SUT is expected to crash
                                print(err_string)
                            else:    
                                # Monitor/Replay crashing is an error on MOBSTA side
                                raise RuntimeError(err_string)

                        # process closed normally
                        else:
                            self.processes.pop(name)
                            self.debug_log_files[name].close()
                            self.debug_log_files.pop(name)


                status_str = "Running [{:.2f}/{}] {}".format(
                    duration, test_time, next(_spin)
                )
                sys.stdout.write(status_str)
                sys.stdout.flush()
                time.sleep(MOBSTA.SLEEP_TIME) 
                duration = time.time() - start_time
                sys.stdout.write("\b"*len(status_str))
                
            if MOBSTA.LOG_REPLAY in self.processes and duration >= test_time:
                warningMsg = ('MOBSTA:\t'
                              'Log still playing after the test has ended.\n'
                              'Current Log Duration {log_duration} + delay {replay_delay}'
                              ' has exceeded test length {test_time}\n'
                              .format(
                                log_duration=max(time.time() - (start_time + replay_delay), 0.0),
                                replay_delay=replay_delay,
                                test_time=test_time)
                              )
                warnings.warn(warningMsg)

        finally:
            self.clean_up()

        print("Test Complete: Results in {}".format(self.test_dir))
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--test-dir',
        '-t',
        help='a directory that holds all the testing information about one project',
        required=True)
    parser.add_argument(
        '--replay-config',
        '-r',
        help='a json configurations file for the log replay including the log replay executable',
        default='replay_config.json')
    parser.add_argument(
        '--monitor-config',
        '-m',
        help='a json configurations file for the monitor that specifies each invaraint and how to spin it up',
        default='monitor_config.json')
    parser.add_argument(
        '--harness-script',
        '-s',
        help='a bash script that runs the SUT',
        default='harness_script.sh')
    parser.add_argument(
        '--log-directory',
        '-l',
        help='a directory to write process logs to. Defaults inside test_dir',
        default='mobsta_logs')
    parser.add_argument(
                 '--seed',
                 help='deterministic unsigned long seed which is passed to the replayer to control random mutations.',
                 default=None)

    mobsta = MOBSTA(parser.parse_args())
    mobsta.run_test()

