# The DROID Robot Platform - Franka FR3 Edition

This repository contains the code for setting up your DROID robot platform for **Franka Research 3 (FR3)** robots and using it to collect teleoperated demonstration data.

Originally designed for Panda robots using polymetis, this fork has been adapted for FR3 with support for the modern [Franky](https://github.com/frankaemika/franky-python) SDK.

### Quick Links

[**[Original DROID Homepage]**](https://droid-dataset.github.io) &ensp; [**[DROID Dataset]**](https://droid-dataset.github.io/dataset.html) &ensp; [**[Paper]**](https://arxiv.org/abs/2403.12945)

![](https://droid-dataset.github.io/droid/assets/index/droid_teaser.jpg)

---------

## Prerequisites

### Hardware
- **Franka Research 3** robot with control desk connected
- **Zed camera** for visual data collection
- **Robotiq 2F gripper** (optional, for grasping operations)
- **NUC PC** (for robot control) - Ubuntu 22.04
- **Laptop** (for teleoperation) - Ubuntu 22.04

### Software
- Python 3.10+
- Franky SDK (`pip install franky-control`)
- pyrobotiqgripper (for Robotiq gripper control)
- dm-control, mujoco, pybullet (for IK solver)

---------

## Getting Started

### 1. Environment Variables

Configure your environment in `droid/misc/parameters.py` or via system environment variables:

```bash
export ROBOT_IP=172.16.0.3           # FR3 robot IP address
export NUC_IP=172.16.0.2             # NUC control PC IP
export LAPTOP_IP=172.16.0.1          # Laptop IP
export ROBOT_TYPE=fr3                # Robot type
```

### 2. Start Robot Server

On the NUC control PC:
```bash
conda activate base
cd scripts/server
./launch_server.sh
```

### 3. Start Teleoperation Client

On the laptop:
```bash
# Use the DROID client to connect to the robot server
```

For more details on the teleoperation interface, see the original DROID documentation.

---------

## Key Differences from Original DROID

| Component | Original (Panda) | FR3 Version |
|-----------|------------------|-------------|
| **Robot Control** | polymetis (USB/PCIe) | [Franky](https://github.com/frankaemika/franky-python) (TCP/IP) |
| **OS** | Ubuntu 18.04 | Ubuntu 22.04 |
| **Python** | 3.7 | 3.10 |
| **Gripper** | Franka gripper | Robotiq 2F (pyrobotiqgripper) |

### New Components

- **FrankyRobot** (`droid/franka/franky_robot.py`) - Main robot control class using Franky SDK
- **IK enhancements** - `cartesian_position_to_joint_position` for direct IK solving
- **Camera improvements** - Skip unavailable cameras during initialization

### Configuration Example

For Robotiq gripper control, set the serial port:
```bash
export GRIPPER_COM_PORT=/dev/ttyUSB0
```

See `droid/misc/parameters.py` for all configurable parameters.

---------

## Docker Setup

Dockerfiles are provided for both NUC and laptop environments. See:
- `.docker/nuc/Dockerfile.nuc` - NUC control PC image
- `.docker/laptop/Dockerfile.laptop` - Laptop teleop image

Use the provided setup scripts:
- `scripts/setup/nuc_setup.sh` - Setup NUC environment
- `scripts/setup/laptop_setup.sh` - Setup laptop environment

---------

## Troubleshooting

**Robot connection issues:**
- Verify `ROBOT_IP` is correct and the robot is accessible
- Check firewall settings allow TCP connections on port 4242

**Gripper not responding:**
- Confirm `GRIPPER_COM_PORT` is correct
- Verify Robotiq gripper is powered on and connected

**Camera not detected:**
- Check Zed SDK installation on laptop
- Run `zed_camera.py` directly to see which cameras are available

For more details, see `docs/setup_records/` for our setup experience.
