# MOBSTA

MOBSTA: Mutation and Oracle Based Safety Testing for Autonomy. 

Robustness testing for your ROS robots!

## License

This repository is provided under LGPL for non-commercial academic
usage. See `LICENSE` for a more details 

For commercial or non-academic usage, a more capable version
is available under proprietary licensing terms. Please contact
robobusters@nrec.ri.cmu.edu for more details. 


## Overview
The MOBSTA program runs end to end robustness tests of a specified
System Under Test (SUT). A Custom Log Replay implementation
reads real log data, mutates the data with the gorgon mutations
library, and then passes it to the SUT. 
SUT output is then monitored for unsafe behavior using defined
saftey invariant scripts. 
 
The system has three parts

- log-replay
- SUT
- Monitor

### Log-replay
The MOBSTA Log Replay API provides a way for MOBSTA to interact with
any log replayer through a python wrapper. Any replay executable
can be added to the MOBSTA log replay API by implementing the 
replay interface and adding the player to the `replay/replay.py`
file. 

MOBSTA itself currently supports two middleware options for log replay:

- [ROS](https://www.ros.org/)
- [ROS2](https://www.ros.org/)

ROS and ROS2 have mutational replay apps that utilize the Gorgon Mutations
Library and can be launched with MOBSTA

#### NOTE:
At the moment, ROS and ROS2 are supported on two seperate 
branches, though we intend to merge them into a single tool the future

#### ROS2 Mutational Replay
ROS2 Log Replay and its dependencies are installed via 
cmake in the deps directory. Log replay should needs a 
valid ROS2 installation to build.

When installing, use the `deps/ros2_install_deps.sh` script
to properly install ROS2 mutational Replay.

### SUT

MOBSTA launches an SUT using a harness script. This allows MOBSTA
launch any SUT without being dependent on the middleware used.

MOBSTA executes the harness script in a seperate process, and 
continuously polls the status of that process during the 
execution of the test

### Monitor

MOBSTA uses a monitor to check the SUT behavior for compliance
with a safety specification. The monitor is a ROS2 node, which 
launches independent "invariant" nodes that check specific 
properties of the SUT. For example, an invariant might be that 
the SUT never commands a speed above a certain speed limit.

Each of these invariants listens to the ROS2 messages output by
the SUT and checks them against the safety rule implemented by 
that invariant. Any violations are then time-stamped and reported 
back to the central monitor node, which logs them to a file.

The current monitor report shows the invariants used, the parameters
for each invariant, and then the timestamps of each violation the
invariants detected. Additionally, each monitor report
provides a metadata section with the replay config that
produced the Monitor report. This way, found violtions can
be easily replicated.

## Example Using ROS2

This is a simple-ish application that is meant to be a sort of
case study for integrating MOBSTA tools with ROS2. This demo uses the
[turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
robot and runs an application which uses the LIDAR to look for a
retroreflector, then drives the robot to park next to it.

### Setup

For this example, you will need to follow the 
`example/turtlebot-example/ROS2_Example.md` file, which 
contains instructions on how to build and run the demo and collect logs. 
In addition, there is an example log file in 
MOBSTA/example/turtlebot-example/logs/ros2 directory

### Running the Example

The **example/turtlebot-test** repository contains a sample **run_mobsta_ros2**
directory for running MOBSTA robustness tests on the parking demo logs. Since 
we don't want to install all the dependencies on the host system for this example
we will mount the MOBSTA directory into the container. 

```
./example/turtlebot-test/run_mobsta_ros2/launch_mobsta_container.sh
```

We can then build the MOBSTA tools for ROS inside the container

```
./deps/install
```

**NOTE**
You only need to run ./deps/install once, as it builds on the host system.
Subsequent launches of the container can simply run MOBSTA

Finally, we can run MOBSTA on the system using the following command

``` 
./run.py -t /path/to/mobsta-turtlebot-test/run_mobsta_ros2 
```

The **run_mobsta_ros2** example directory is configured to check two
invariants. There’s a “valid numbers invariant” that just makes sure
that the commanded velocities never contain inf or NaN values. And there’s
a “correct commands invariant” that checks to make sure that the parking demo
node is executing the correct step of rotate-move-rotate-move for the 
current state of the environment (the example/turtlebot-example/Demo_Explanation.md
file contains an overview of the steps that the node executes). The 
replay_config.json file is set up so that the “intensities” component of the  
/scan messages are decrease by a fixed amount. This could correlate to an 
underpowered or failing laser diode, or with a miss configured sensor. 
This reduced scan intensity will cause the parking demo to fail to identify 
a parking spot, and thus trigger a failure of the CorrectCommands invariant,
so that when you run MOBSTA you get an output like the following:

``` 
{
    "Invariants": [
        {
            "invariant_name": "SUT Crash",
            "violations": [],
            "config": {}
        },
        {
            "invariant_name": "ValidNumbersInvariant",
            "violations": [],
            "config": {
                "invariant_name": "ValidNumbersInvariant",
                "invariant_file": "${HOME}/MOBSTA/example/turtlebot-example/run_mobsta_ros2/invariants/valid_numbers_invariant_ros2.py",
                "invariant_params": {
                    "velocity_topic": "/cmd_vel"
                }
            }
        },
        {
            "invariant_name": "CorrectCommandsInvariant",
            "violations": [
                "3.239122152328491",
                "3.342866897583008",
                ...
                "53.853010416030884"
            ],
            "config": {
                "invariant_name": "CorrectCommandsInvariant",
                "invariant_file": "${HOME}/MOBSTA/example/turtlebot-example/run_mobsta_ros2/invariants/correct_commands_invariant_ros2.py",
                "invariant_params": {
                    "velocity_topic": "/cmd_vel",
                    "joint_states_topic": "/joint_states",
                    "bad_command_limit": 10,
                    "angle_goal_deg": 5.0,
                    "position_goal": 0.05,
                    "distance_goal": 0.5,
                    "angle_uncertainty_deg": 3.0,
                    "position_uncertainty": 0.2,
                    "parking_spot_x_1": 0.285,
                    "parking_spot_y_1": 0.815,
                    "parking_spot_x_2": -0.185,
                    "parking_spot_y_2": 0.986
                }
            }
        }
    ],
    "Metadata": {
        "replay-type": "ROS2",
        "replay-log": "${HOME}/MOBSTA/example/turtlebot-example/logs/ros2/2023-06-09_sensor_inputs",
        "replay-delay": 0.0,
        "test-time": 90.0,
        "mutation-config": {
            "topics": {
                "/scan": [
                    {
                        "message_intercept": "/intensities",
                        "mutations": [
                            {
                                "mutation_type": "Float32Array_AddToWholeArrayMutator",
                                "mutation_chance": 1.0,
                                "timeframe_begin": 0.0,
                                "timeframe_end": -1,
                                "mutation_args": {
                                    "valueToAdd": -100.0,
                                    "minArrayValue": 0.0,
                                    "maxArrayValue": 255.0,
                                    "arraySize": 259
                                }
                            }
                        ]
                    }
                ]
            }
        },
        "seed": 1787126726
    }
}
```

The output is divided into two sections. The bottom section, "Metadata", gives
a summary of the log that was used and the mutators. The top section, 
"Invariants", tells you if any of the invariants were violated. If they were,
then the violation times will show up in the "violations" array. In the above
example, the "CorrectCommands" invariant is violated from the 3 second mark until
the 54 second mark (at which point the turtlebot has reached the parking spot 
in the replayed log and the correct action is to do nothing).


## Acknowledgements

This material is based upon work supported by the U.S. Army Research Office and the U.S. Army Futures Command under Contract No. W911NF-20-D-0002.

The content of the information does not necessarily reflect the position or the policy of the government and no official endorsement should be inferred.
