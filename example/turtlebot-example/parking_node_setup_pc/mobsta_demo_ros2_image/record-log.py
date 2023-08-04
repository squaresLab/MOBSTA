import os
import time
import subprocess
import signal
import shlex
import sys
from datetime import datetime

class CommandRunner:
    def __init__(self, command, logging_dir):
        try:
            self.process = subprocess.Popen(shlex.split(command), cwd = logging_dir)
        except Exception as err:
            failure_msg = "Running [%s] failed: %s"%(command, str(err))
            print(failure_msg)
            self.process = None

    def stop(self):
        if self.process is None:
            return
        self.process.terminate()
        term_start = time.time()
        while math.fabs(time.time() - term_start) < 2.0:
            self.process.communicate()
            if self.process.returncode is not None:
                break
            time.sleep(0.1)
        if self.process.returncode is not None:
            self.process.kill()

global kill_signal_received
kill_signal_received = False
def kill_signal(signum, stuff):
    print("Got kill signal %d\n"%signum)
    global kill_signal_received
    kill_signal_received = True

if __name__ == "__main__":
    # Get the directory for writing the logs
    logging_dir = os.path.join(os.getenv("HOME"), "logs/ros2")
    
    # Generate date folder
    now = datetime.now()
    date_folder = now.strftime("%Y-%m-%d")
    logging_dir = os.path.join(logging_dir, date_folder)
    if not os.path.exists(logging_dir):
        print("Creating directory %s"%logging_dir)
        os.makedirs(logging_dir)
        
    # Generate log folder
    if len(sys.argv) == 1:
        log_name_folder = now.strftime("%Y-%m-%d-%H-%M-%S")
    else:
        log_name_folder = sys.argv[1]
    logging_dir = os.path.join(logging_dir, log_name_folder)
    print("Creating directory %s"%logging_dir)
    os.makedirs(logging_dir)
    
    # Generate the record commands
    recording_command_inputs = "ros2 bag record -o sensor_inputs /scan /tf_static /imu /sensor_state /joint_states /magnetic_field /odom /tf"
    recording_command_control_outputs = "ros2 bag record -o control_outputs /cmd_vel"
    
    # Run the commands
    commands = [recording_command_inputs, recording_command_control_outputs]
    processes = []
    for command in commands:
        new_process = CommandRunner(command, logging_dir)
        processes.append(new_process)
        
    signal.signal(signal.SIGTERM, kill_signal)
    signal.signal(signal.SIGHUP, kill_signal)

    while not kill_signal_received:
        print("Press ctl-c to stop recording.")
        time.sleep(1.0)        
    
    for process in processes:
        process.stop()
