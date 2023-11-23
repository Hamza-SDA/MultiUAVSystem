# UAS Offboard Planner<!-- omit in toc -->

**IMPORTANT NOTE:** This repository has been renamed from `quas_vicon` to `uas_offboard_planner`.

Please update your ROS source code to correctly refer to the new package name as well as the git URL in your local machine as follows:

If using HTTPS:

```sh
git remote set-url origin https://gitlab.com/qut-asl-upo/UAS_offboard_planner.git
```

If using SSH:

```sh
git remote set-url origin git@gitlab.com:qut-asl-upo/UAS_offboard_planner.git
```

---

A guide to enabling offboard flight using either a VICON or vision based (e.g ZED2 or Intel T265) positioning system with PX4 enabled UAV systems and a companion computer.
This software is to be installed and executed on the companion computer connected to the auto-pilot via an UART connection.

The following guide assumes that you are using a ROS-Kinetic or ROS-Melodic enabled system and with a UAV using PX4 (1.9.2) firmware.

This package and the following steps are designed to enable position and velocity based offboard control when using a VICON or vision system for localisation.

## Table of Contents<!-- omit in toc -->

- [Installation](#installation)
  - [1. Permissions](#1-permissions)
  - [2. Dependencies](#2-dependencies)
    - [ROS](#ros)
    - [MAVROS](#mavros)
    - [VRPN Client [VICON Users]](#vrpn-client-vicon-users)
    - [Octomap](#octomap)
  - [3. Catkin Workspace](#3-catkin-workspace)
- [Setup](#setup)
  - [1. UAV and Ground Control Station](#1-uav-and-ground-control-station)
  - [2. Flight Controller Settings](#2-flight-controller-settings)
- [Getting Started](#getting-started)
  - [Using the flight setup file](#using-the-flight-setup-file)
  - [Using the action sequence file](#using-the-action-sequence-file)
- [Contributing](#contributing)
- [Changelog](#changelog)
- [Copyright](#copyright)

## Installation

### 1. Permissions

Ubuntu comes with a serial modem manager which interferes heavily with any robotics related use of a serial port (or USB serial).
It can be uninstalled without side effects:

1. Open a terminal and type:

    ```sh
    sudo apt-get remove modemmanager -y
    ```

2. The user needs to be part of the group `dialout`:

    ```sh
    sudo usermod -a -G dialout $USER
    ```

3. **Logout and login again** to enable the change to user permissions.

### 2. Dependencies

#### ROS

Ensure you have a working installation of ROS.
Follow [this link](http://wiki.ros.org/Installation/Ubuntu) with instructions to install a distribution.

#### MAVROS

Install the MAVROS (MAVLink on ROS) package.
This enables MAVLink communication between computers running ROS, MAVLink enabled autopilots, and MAVLink enabled GCS.

1. Open a terminal and install GeographicLib:

    ```sh
    sudo apt install geographiclib-tools -y
    ```

2. Install the GeographicLib dataset:

    ```sh
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

    sudo bash install_geographiclib_datasets.sh

    rm install_geographiclib_datasets.sh
    ```

3. Install MAVROS from the package manager:

    ```sh
    sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
    ```

#### VRPN Client [VICON Users]

A VRPN Client needs to be started in order to republish the **VICON** readings in a compatible format.

1. If you have not installed the VRPN client do so by running:

    ```sh
    sudo apt install ros-$ROS_DISTRO-vrpn-client-ros
    ```

#### Octomap

Run the following in a shell terminal to install octomap:

```sh
sudo apt-get install ros-$ROS_DISTRO-octomap-*
```

### 3. Catkin Workspace

Clone the package into a ROS Catkin Workspace and run `catkin build` in the upper workspace directory.

```sh
cd /path/to/catkin_ws
cd src
git clone https://gitlab.com/qut-asl-upo/UAS_offboard_planner.git
git clone https://github.com/pal-robotics/aruco_ros.git
cd ..
catkin build uas_offboard_planner -DCMAKE_BUILD_TYPE=Release
```

## Setup

### 1. UAV and Ground Control Station

1. Install QGroundControl in your local machine using [these](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#download-and-install) instructions.

2. Open QGroundControl, connect your FCU using a USB cable and flash the supported firmware [v1.9.2](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.9.2) following [these](https://docs.px4.io/master/en/config/firmware.html#installing-px4-master-beta-or-custom-firmware) instructions.

3. Recalibrate your vehicle sensors using QGroundControl.

### 2. Flight Controller Settings

Once you have flashed the new firmware you will need to run through the necessary calibration steps for your UAV and the px4 firmware.
Connect the UAV to QGroundControl to perform these calibration steps.

Then a number of flight controller parameters need to be edited within the QGroundControl software in order to enable VICON based localisation.

```txt
EKF2_EV_DELAY = 50 // Default 175
```

```txt
EKF2_AID_MASK = 24 // Enable Bits 3 and 4 (Vision Position and Vision Yaw Fusion) and Disable GPS Fusion
```

```txt
EKF2_HGT_MODE = 3 // Disable Barometric Height and Enable Vision Height
```

```txt
RTL_RETURN_ALT = 2 //These edit the failsafe mode to prevent crashing into the ceiling
```

```txt
RTL_DESCEND_ALT = 2
```

## Getting Started

Once you have performed the necessary installation and setup instructions it will be possible to use the package to control a UAV in a VICON enabled environment.

The package contains two nodes:

- [`offboard_autoland.cpp`](src/offboard_autoland.cpp)
- [`offboard_control.cpp`](src/offboard_control.cpp)

Both are necessary for safe operation.

The launch procedure for a UAV in the DVP flight test area is as follows (run each of these in their own terminal)

First launch the vprn client, onboard is preferred, but it can be run on the GCS.

```sh
roslaunch uas_offboard_planner vrpn.launch
```

Then launch the MAVROS controller node. This is generally platform specific. When using the intel Aero platform run the following onboard

```sh
rosrun mavros mavros_node _fcu_url:=tcp://127.0.0.1:5760 _system_id:=2
```

Then proceed to launch the `uas_offboard_planner` autoland node.
This node is a failsafe node.
When running, pressing the enter key within the active window will force the drone to switch to auto-land mode.
Although this should not be necessary, if the UAV performs an unintended action or behaves erratically it is wise to force a landing and investigate what could be causing issues.

```sh
rosrun uas_offboard_planner offboard_autoland
```

Before the UAV can be launched, it is advised to run the following commands in their own terminals to check the localisation.

```sh
rostopic echo /mavros/vision_pose/pose
rostopic echo /mavros/local_position/pose
```

These topics must be echoing similar position values (within one or two centimetres) or the localisation is experiencing issues and a flight-test is unsafe.

Finally, launch the control node. This control node uses a text file to fetch user programmed commands (outlined further below).

```sh
roslaunch uas_offboard_planner hardware_control.launch
```

Remember to reactivate the autoland window in the event an emergency landing is required and prepare for UAV takeoff.

### Using the flight setup file

The [offboard_control](src/offboard_control.cpp) ROS node reads *.yaml configuration files that define the flight setup conditions.

The default file is [default_dvp.yaml](config/default_dvp.yaml), but you can duplicate or modify the file and call the new filename from [hardware_control.launch](launch/hardware_control.launch) if working under different conditions.

The settings comprise:

```yaml
# This file stores ROS params of the flight setup.

flight_setup:
  # Lower and Upper boundaries in metres (local position).
  limit_x:
    min: -2.0
    max: 2.0
  limit_y:
    min: -2.0
    max: 2.0
  limit_z:
    min: 0.4
    max: 2.5

  # Publishing Frequency in Hz.
  offboard_freq: 20.0

  # Local takeoff altitude (in metres) before broadcasting initial waypoint.
  takeoff_altitude: 1.5
  # Position tolerance (in metres) between actual UAV position and next
  # position waypoint.
  waypoint_error: 0.25

  # Local UAV pose topic to subscribe.
  pose_topic: "/mavros/local_position/pose"

```

### Using the action sequence file

The UAV is commanded using any of the sequence of action commands declared in the [parameters](parameters) folder (default file: [hover_20sec.txt](parameters/hover_20sec.txt)).
**THE FILE MUST BE FORMATTED CORRECTLY.**

The text file is layed out as below, with the major parameters in the first line and each control command layed out in each line below it:

If using **position** commands:

```txt
pos 'coord_x(m)' 'coord_y(m)' 'coord_z(m)' 'yaw(deg)' 'duration(s)'
```

If using **velocity** commands:

```txt
vel 'vel_x(m/s)' 'vel_y(m/s)' 'coord_z(m)' 'yaw_rate(deg_s)' 'duration(s)'
```

If using **dual** commands:

```txt
dual 'coord_x(m)' 'coord_y(m)' 'coord_z(m)' 'xy_vel(m/s)' 'yaw(deg)' 'duration(s)'
```

**NOTE:** When using **dual** commands, the node will send position commands with a *truncated* maximum horizontal velocity by overriding the `MPC_XY_VEL_MAX` FCU parameter.

Below is an example sequence:

```txt
pos -1.0 0.0 1.5 90 3 // position_command, x = -1.0 m, y = 0.0 m, z = 1.5 m, yaw = 90 deg, duration = 3 s

vel 0.5 0 1.5 0 1 // velocity_command, x = 0.5 m/s, y = 0 m/s, z = 1.5 m (hold z height), yaw_rate = 0 deg/s, duration = 1 s

dual 0.0 0.0 1.5 0.5 -75 3 // pos_and_vel_cmd, x = 0.0 m, y = 0.0 m, z = 1.5 m, hor_vel = 0.5 m/s, yaw = -75 deg, duration = 3 s
```

When given a position command the UAV moves within the local frame using metres as the distance, and degrees as the rotation.

When given a velocity command the UAV moves within the UAV frame (+x is forward, +y = left) using $`m/s`$ as the velocity and $`^{\circ}/s`$ as the yaw rate.

The Z Command is used to provide a Z hold position, as in the position command.

## Contributing

Please check our contributing [guidelines](CONTRIBUTING.md) for further details.

## Changelog

See the [changelog](CHANGELOG.md) for further details.

## Copyright

Copyright &copy; 2018-2021 Fernando Vanegas (f.vanegasalvarez@qut.edu.au), Juan Sandino (sandinoj@qut.edu.au), Felipe Gonzalez (felipe.gonzalez@qut.edu.au), Queensland University of Technology.
All rights reserved.

This software is protected under the Copyright Act 1968 and the
QUT MOPP F/5.1 (Copyright).
For authorised uses only.

**This software is confidential and cannot not be used, modified or shared unless for the approved specific purposes by the Copyright owners.**
