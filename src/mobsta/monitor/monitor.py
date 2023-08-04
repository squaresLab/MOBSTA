#!/usr/bin/env python
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

import signal
import sys
import os
import copy
import rospy
import argparse
import json
import subprocess
import time
import tempfile
from diagnostic_msgs.msg import KeyValue

# time in seconds
INVARIANT_CHECK = 1.0


class Monitor:
    """Monitors the invariants and records their reported violations. finally,
    collects the data reported and writes them to a JSON file for later use.

    When shutdown signal is received, The Monitor interprets that as the
    program has run successfully, and it starts up shutdown sequence including
    generating the monitor report. usually shutdown signal is a mask for both
    SIGINT and SIGTERM but in this case it only masks SIGTERM bacause SIGINT
    handler is overridden to be handled separatly

    when SIGINT is received, the monitor interprets that as the SUT has crashed.
    It overrides the regular shutdown behaviour and forwards the signal to the
    Crash Invariant to report a crash.

    Args:
        config_file (str): The file that defines which invariant monitor to
            start and how to run it.
        test_dir (str): A dir path that the test directory has all the
            information regarding a single SUT.
        metadata (file): json file with data that will be tacked on to the 
            monitor report

    """

    monitor_report = {}

    def __init__(self, config_file, test_dir, metadata_file):  # noqa: D107
        with open(config_file, 'r') as fc:
            config_str = fc.read()
        config_json = json.loads(config_str)

        if metadata_file is not None:
            with open(metadata_file, 'r') as fc:
                metadata_str = fc.read()
            self.metadata_json = json.loads(metadata_str)
        else: 
            self.metadata_json = None

        crash_node_file_path = os.path.dirname(os.path.abspath(__file__)) + '/crash_invariant.py'
        self.crash_node = subprocess.Popen(['python', crash_node_file_path, 'CrashInvariant'],
                                           stdout=subprocess.PIPE, 
                                           stderr=subprocess.PIPE)
        Monitor.monitor_report['CrashInvariant'] = {
            'violations': [],
            'config': {}
        }

        self.invariant_nodes = {'CrashInvariant': self.crash_node}
        for invariant in config_json:
            invariant_name = invariant['invariant_name']
            Monitor.monitor_report[invariant_name] = {
                'violations': [],
                'config': invariant
            }

            invariant_file = os.path.expandvars(invariant['invariant_file'])
            try:
                env = copy.copy(os.environ)
                env["PYTHONPATH"] = ":".join([
                    env["PYTHONPATH"],
                    os.path.dirname(callable_path())
                    ])

                with tempfile.NamedTemporaryFile(mode='w', delete=False) as param_file:
                    json.dump(invariant["invariant_params"], param_file)
                    self.invariant_nodes[invariant_name] = subprocess.Popen(
                        [invariant_file, invariant_name, '-p', param_file.name],
                        env=env,
                        stdout=subprocess.PIPE, 
                        stderr=subprocess.PIPE)
            except OSError:
                self.clean_up()
                raise RuntimeError(
                    "Could not run invariant {} with path {}".format(
                        invariant_name, 
                        invariant_file,
                    )
                )
            except: 
                self.clean_up()
                raise

        report_file_name = time.strftime('%Y%m%d_%H%M%S') + '.json'
        report_dir = os.path.join(os.path.abspath(test_dir), 'monitor_reports')
        if not os.path.exists(report_dir):
            os.makedirs(report_dir)
        self.report_path = os.path.join(report_dir, report_file_name)

    def callback(self, msg):
        """Stores the data recieved by different invariants.

        Args:
            msg (diagnostic_msgs/KeyValue{Key,Value}): time stamps associated
              of the violations
            Key: The Invariant Name that reports the violation
            Value: The timestamp of when that violation happened

        Returns:
            None

        """
        Monitor.monitor_report[msg.key]['violations'].append(msg.value)

    def handle_sigint(self, sig, frame):
        """Handles the Signal triggered by the SUT failure and pass it to the invariant node,
        Then shutdown rspy and exit the program

        Args:
            sig (int): signal numner requied for a signal handler
            frame (frame object): the current stack frame required for a signal handler

        Returns:
            None
        """
        self.crash_node.send_signal(signal.SIGINT)
        rospy.signal_shutdown('SUT crashed')

    def get_json_report(self):
        """Converts the monitor_report dict to a json formated properly. then
        writes the json to a file contianing the full report.

        Returns:
            None
        """
        results = []
        for inv_name, inv_data in Monitor.monitor_report.items():
            result = {
                'invariant_name': inv_name,
                'violations': inv_data['violations'],
                'config': inv_data['config']
            }
            results.append(result)

        report = {
            "Invariants": results
        }

        if self.metadata_json is not None:
            report["Metadata"] = self.metadata_json 
        
        report_str = json.dumps(report, indent=4)

        # parse the results json
        with open(self.report_path, 'w') as fr:
            fr.write(report_str)

    def close(self):
        """Closes all open nodes and generates the .json report file, Runs
        when the rosnode triggers shutdown().

         Returns:
            None
        """
        self.clean_up()

        self.get_json_report()
    
    def clean_up(self):
        """Closes all open nodes.

        Returns:
            None
        """
        for node in self.invariant_nodes.values():
            node.terminate()
            
            # Flush the invariant stderrs to the monitor stderr
            node_stderr = node.stderr.read()
            sys.stderr.write(node_stderr.decode())

    def check_invariants_running(self):
        """Runs a health check on all the invariants to ensure that the are
            running. if any of the invariants crashed then the monitor will
            crash too and report it to HPSTA

        Returns:
            None
        """
        for node_name, node in self.invariant_nodes.items():
            node.poll()

            ret = node.returncode
            if ret is not None:
                node_stderr = node.stderr.read()

                # Always copy stderr from each Invariant to the monitor log 
                #   by printing it to stderr
                sys.stderr.write(node_stderr.decode())
                if ret != 0:
                    error_message = '{node_name} Error: {error}'.format(
                        node_name=node_name, error=node_stderr.decode())
                    self.invariant_nodes.pop(node_name)
                    self.close()
                    raise RuntimeError(error_message)


def main(args):
    if args.delay > 0:
        print("sleeping for {} seconds before startup".format(args.delay))
        time.sleep(args.delay)

    rospy.init_node('monitor', anonymous=False)

    monitor = Monitor(args.config_file, args.test_dir, args.metadata_file)
    signal.signal(signal.SIGINT, monitor.handle_sigint)

    rospy.Subscriber('/violations', KeyValue, monitor.callback)
    rospy.on_shutdown(monitor.close)
    while not rospy.is_shutdown():
        monitor.check_invariants_running()
        try: 
            rospy.sleep(INVARIANT_CHECK)
        except rospy.exceptions.ROSInterruptException:
            pass


def callable_path():
    """Get an executable path to the monitor.

    __file__ may return a .pyc, but we always want the .py, 
    because that is the one that is chmod'd

    Returns:
        str: The callable path to this monitor
    """
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "monitor.py")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file')
    parser.add_argument('--test-dir', required=True)

    parser.add_argument('--metadata-file', 
                        help='json file with extra metadata which will be included in the report', 
                        default=None)
    parser.add_argument('--delay',
                        help='seconds to delay monitor startup in case roscore is restarted',
                        type=float,
                        default=0.0)
    main(parser.parse_args())
