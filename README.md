# The DROID Robot Platform

This repository contains the code for setting up your DROID robot platform and using it to collect teleoperated demonstration data. This platform was used to collect the [DROID dataset](https://droid-dataset.github.io), a large, in-the-wild dataset of robot manipulations.

If you are interested in using the DROID dataset for training robot policies, please check out our [policy learning repo](https://github.com/droid-dataset/droid_policy_learning).
For more information about DROID, please see the following links: 

[**[Homepage]**](https://droid-dataset.github.io) &ensp; [**[Documentation]**](https://droid-dataset.github.io/droid) &ensp; [**[Paper]**](https://arxiv.org/abs/2403.12945) &ensp; [**[Dataset Visualizer]**](https://droid-dataset.github.io/dataset.html).

![](https://droid-dataset.github.io/droid/assets/index/droid_teaser.jpg)

---------
## Setup Guide

We assembled a step-by-step guide for setting up the DROID robot platform in our [developer documentation](https://droid-dataset.github.io/droid).
This guide has been used to set up 18 DROID robot platforms over the course of the DROID dataset collection. Please refer to the steps in this guide for setting up your own robot. Specifically, you can follow these key steps:

1. [Hardware Assembly and Setup](https://droid-dataset.github.io/droid/docs/hardware-setup)
2. [Software Installation and Setup](https://droid-dataset.github.io/droid/docs/software-setup)
3. [Example Workflows to collect data or calibrate cameras](https://droid-dataset.github.io/droid/docs/example-workflows)

If you encounter issues during setup, please raise them as issues in this github repo.

---------
## Franka FR3 Adaptation

This fork has been adapted for **Franka Research 3 (FR3)** robots, replacing the original Panda-specific dependencies with a more flexible, modern implementation.

### Key Changes

| Component | Original | FR3 Version |
|-----------|----------|-------------|
| **Robot Control SDK** | polymetis (USB/PCIe) | [Franky](https://github.com/frankaemika/franky-python) (TCP/IP) |
| **Base OS** | Ubuntu 18.04 | Ubuntu 22.04 |
| **Python** | 3.7 | 3.10 |

### New Features

- **FrankyRobot** class (`droid/franka/franky_robot.py`) - TCP/IP control of FR3
- **Robotiq 2F gripper** support via `pyrobotiqgripper`
- **IK solver enhancements** - `cartesian_position_to_joint_position` method
- **Zed camera robustness** - Skip unavailable cameras gracefully

### Configuration

Configuration for FR3 is done via environment variables:

```
ROBOT_IP=172.16.0.3                 # FR3 robot IP address
GRIPPER_COM_PORT=/dev/ttyUSB0       # Robotiq gripper serial port (optional)
```

See `droid/misc/parameters.py` for all configurable parameters.
