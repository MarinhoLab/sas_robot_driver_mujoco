![Static Badge](https://img.shields.io/badge/status-experimental-critical)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![Static Badge](https://img.shields.io/badge/ROS_2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/MuJoCo-3.16-orange)

# sas_robot_driver_mujoco

A [SAS](https://smartarmstack.github.io/) robot driver for robots in [MuJoCo](https://mujoco.org) scenes, based on
[cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco). The simulation runs inside the driver, in real
time, so a robot in a MuJoCo scene has the same ROS 2 interface as a real robot with a SAS driver
(`get/joint_states`, `set/target_joint_positions`, ...).

> [!WARNING]
> This project is under active development.

## How it works

`sas::RobotDriverMujoco` is a `sas::RobotDriver` run by `sas::RobotDriverROS`:

| RobotDriver | MuJoCo |
|---|---|
| `connect()` | Loads the scene (`model_path`), and opens its window (unless `headless`) |
| `initialize()` | Resets the scene (to `home_keyframe`, if given) and starts the simulation in real time |
| `get_joint_positions()`, `get_joint_velocities()`, `get_joint_torques()` | The state of the joints `robot_joint_names` |
| `set_target_joint_positions()`, `set_target_joint_velocities()`, `set_target_joint_torques()` | The commands of the actuators of those joints, which must be of the kind of the command (position actuators, velocity actuators, motors) |
| `deinitialize()` | Stops the simulation |
| `disconnect()` | Closes the scene |

Shutdown uses `sas::ShutdownSignaler` (`sas_core/sas_shutdown_signaler.hpp`): a SIGINT stops the driver, and so does
closing the window of the scene (through the control loop callback of `sas::RobotDriver`, which `RobotDriverROS` runs at
each iteration).

## Parameters

| Parameter | | |
|---|---|---|
| `model_path` | required | The MJCF file of the scene |
| `robot_joint_names` | required | The joints of the robot, in the order of `q` |
| `joint_limits_min`, `joint_limits_max` | required | Joint limits, in degrees |
| `thread_sampling_time_sec` | required | Period of the control loop of `RobotDriverROS` |
| `home_keyframe` | optional (`""`) | Keyframe to start from (`""`: the initial state of the model) |
| `headless` | optional (`false`) | Run without a window |

The node name is the prefix of the topics (e.g., `/ur3e/get/joint_states`).

## Build

Prerequisites: ROS 2 Jazzy, SAS, [DQ Robotics](https://github.com/dqrobotics/cpp), and
[cpp-interface-mujoco](https://github.com/juanjqo/cpp-interface-mujoco) 1.1 (installed; it is not a ROS package). If
cpp-interface-mujoco is not installed in a system path, add its prefix to `CMAKE_PREFIX_PATH` and `LD_LIBRARY_PATH`.

```shell
cd ~/ros2_ws/src
git clone https://github.com/MarinhoLab/sas_robot_driver_mujoco
cd ~/ros2_ws
colcon build --packages-select sas_robot_driver_mujoco && source install/setup.bash
```

## Example: a UR3e

With the UR3e of [cpp-interface-mujoco-robots](https://github.com/juanjqo/cpp-interface-mujoco-robots) (parameters in
[`config/ur3e.yaml`](config/ur3e.yaml)):

```shell
ros2 launch sas_robot_driver_mujoco robot_launch.py model_path:=<cpp-interface-mujoco-robots>/models/universal_robots_ur3e/scene.xml
```

```shell
ros2 topic echo /ur3e/get/joint_states
ros2 topic pub --once /ur3e/set/target_joint_positions std_msgs/msg/Float64MultiArray "data: [0.5, -1.2, 1.0, -0.6, -1.0, 1.0]"
```

## Example: a Unitree Z1

With the Z1 (without its gripper) of [cpp-interface-mujoco-robots](https://github.com/juanjqo/cpp-interface-mujoco-robots)
(parameters in [`config/z1.yaml`](config/z1.yaml), chosen by `name:=z1`):

```shell
ros2 launch sas_robot_driver_mujoco robot_launch.py name:=z1 model_path:=<cpp-interface-mujoco-robots>/models/unitree_z1/z1.xml
```

```shell
ros2 topic echo /z1/get/joint_states
ros2 topic pub --once /z1/set/target_joint_positions std_msgs/msg/Float64MultiArray "data: [0.5, 1.2, -0.8, 0.6, 0.7, 1.0]"
```

## Launch arguments

| Argument | Default | |
|---|---|---|
| `model_path` | required | The MJCF file of the scene |
| `name` | `ur3e` | The name of the node, which is also the prefix of its topics |
| `config_file` | `config/<name>.yaml` of this package | The parameters of the robot (e.g., `config/ur3e.yaml`, `config/z1.yaml`) |
| `headless` | `false` | Run without a window |

## License

[LGPL-3.0-or-later](LICENSE), which supplements the [GPL-3.0](COPYING).
