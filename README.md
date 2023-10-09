# MOBSTA

MOBSTA: Mutation Based Safety Testing for Autonomy.

Robustness testing for your ROS robots!

## License

This repository is provided under LGPL for non-commercial academic
usage.

For commercial or non-academic usage, a more capable version
is available for under proprietary licensing terms. Please contact
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
- [ROS 2](https://www.ros.org)

ROS and ROS 2 have mutational replay apps that utilize the Gorgon Mutations
Library and can be launched with MOBSTA

#### NOTE

At the moment, ROS and ROS 2 are supported on two seperate
branches, though we intend to merge them into a single tool the future

#### ROS Mutational Replay

ROS Log Replay and its dependencies are installed via
cmake in the deps directory. Log replay should needs a
valid ROS installation to build.

[NLohmann's cpp-json](https://github.com/nlohmann/json) is
downloaded via CMake fetchContent and requires an internet connection.

When installing, use the `deps/ros_install_deps.sh` script
to properly install ROS mutational Replay.

### SUT

MOBSTA launches an SUT using a harness script. This allows MOBSTA
launch any SUT without being dependent on the middleware used.

MOBSTA executes the harness script in a seperate process, and
continuously polls the status of that process during the
execution of the test

### Monitor

MOBSTA uses a monitor to check the SUT behavior for compliance
with a safety specification. The monitor is a ROS node, which
launches independent "invariant" nodes that check specific
properties of the SUT. For example, an invariant might be that
the SUT never commands a speed above a certain speed limit.

Each of these invariants listens to the ROS messages output by
the SUT and checks them against the safety rule implemented by
that invariant. Any violations are then time-stamped and reported
back to the central monitor node, which logs them to a file.

The current monitor report shows the invariants used, the parameters
for each invariant, and then the timestamps of each violation the
invariants detected. Additionally, each monitor report
provides a metadata section with the replay config that
produced the Monitor report. This way, found violtions can
be easily replicated.

## Example Using ROS

This is a simple-ish application that is meant to be a sort of
case study for integrating MOBSTA tools with ROS 1. This demo uses the
[TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
robot and runs an application which uses the LIDAR to look for a
retroreflector, then drives the robot to park next to it.

### Setup

For this example, you will need to follow the
`example/turtlebot-example/ROS1_Example.md` file, which
contains instructions on how to build and run the demo and collect logs.
In addition, there are some example log files in this directory
project NAS in the directory /mobsta/turtlebot3_logs/ros1.


### Running the Example

The **example/turtlebot-test** repository contains a sample **run_mobsta_ros1**
directory for running MOBSTA robustness tests on the parking demo logs. Since
we don't want to install all the dependencies on the host system for this example
we will mount the MOBSTA directory into the container.

```shell
./example/turtlebot-test/run_mobsta_ros1/launch_mobsta_container.sh
```

We can then run MOBSTA within the container via the following command:

```shell
cd ~/MOBSTA
./run.py -t /path/to/mobsta-turtlebot-test/run_mobsta_ros1
```

#### Running MOBSTA outside the container

It is also possible to run MOBSTA on the host system. This can be desirable when
one expects the SUT build, and thus Docker image, to change, but wishes to maintain
the test setup. In this case run the install on the host system.

The **run_mobsta_ros1** example directory is configured to check two
invariants. There’s a “valid numbers invariant” that just makes sure
that the commanded velocities never contain inf or NaN values. And there’s
an “all steps invariant” that checks to make sure that the parking demo
node runs all the steps of rotate-move-rotate-move (The mobsta-turtlebot-test
repo contains an overview of the steps that the node executes). The
replay_config.json file is set up so that the “x” component of the odometry
pose gets stuck at zero. In most cases, this should cause the AllStepsInvariant
to trigger a failure, so that when you run MOBSTA you get an output like the
following:

```json
{
    "Invariants": [
        {
            "invariant_name": "CrashInvariant", "violations": [], "config": {}
        }, {
            "invariant_name": "ValidNumbersInvariant", "violations": [],
            "config": {
                "invariant_name": "ValidNumbersInvariant", "invariant_file":
                "${HOME}/src/mobsta/mobsta-turtlebot-test/run_mobsta_ros1/invariants/valid_numbers_invariant.py",
                "invariant_params": {
                    "velocity_topic": "/cmd_vel"
                }
            }
        }, {
            "invariant_name": "AllStepsInvariant", "violations": [
                "70.1544554233551", "71.15441966056824", "72.1544086933136",
                "73.1543710231781", "74.15439224243164", "75.15440082550049",
                "76.15441346168518", "77.15437936782837", "78.15440225601196",
                "79.1543984413147", "80.15437817573547", "81.15410232543945",
                "82.1543915271759"
            ], "config": {
                "invariant_name": "AllStepsInvariant", "invariant_file":
                "${HOME}/src/mobsta/mobsta-turtlebot-test/run_mobsta_ros1/invariants/all_steps_invariant.py",
                "invariant_params": {
                    "velocity_topic": "/cmd_vel", "time_limit": 70.0
                }
            }
        }
    ], "Metadata": {
        "replay-type": "ROS", "replay-log":
        "${HOME}/src/mobsta/logs/ros1/2023-06-01/static_demo/sensor_inputs_2023-06-01-11-37-28_0.bag",
        "replay-delay": 0.0, "test-time": 90.0, "mutation-config": {
            "topics": {
                "/odom": {
                    "message_intercept": "/pose/pose/position/x", "mutations":
                    [
                        {
                            "mutation_type": "Double_StuckValueMutator",
                            "mutation_chance": 1.0, "timeframe_begin": 0.0,
                            "timeframe_end": -1, "mutation_args": {
                                "stuckValue": 0.0
                            }
                        }
                    ]
                }
            }
        }, "seed": 2184321993
    }
}
```

The output is divided into two sections. The bottom section, "Metadata", gives
a summary of the log that was used and the mutators. The top section,
"Invariants", tells you if any of the invariants were violated. If they were,
then the violation times will show up in the "violations" array. In the above
example, the "All Steps Invariant" was configured with a time limit of 70
seconds, so that it would trigger if the demo did not execute all of the steps
within 70 seconds. The "violations" array under "AllStepsInvariant" shows that
this invariant was violated after the 70 second mark with the stuck value
mutator active.

## Development

### Running Unit/integration Tests

unittests depend on nose2, to install run:

```shell
python -m pip install nose2
```

then run the following command

```shell
python -m nose2 -v --top-level-directory $PWD/src/MOBSTA
```

## Acknowledgements

This material is based upon work supported by the U.S. Army Research Office and the U.S. Army Futures Command under Contract No. W911NF-20-D-0002.

The content of the information does not necessarily reflect the position or the policy of the government and no official endorsement should be inferred.
