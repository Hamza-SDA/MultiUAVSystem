# Changelog

## v0.3.0 (2021-04-16)

### Features

- BREAKING CHANGE: Rename ROS package to `uas_offboard_planner`.
- Add standalone versions of *motion server* and *observation server*.
- Update README.md, CONTRIBUTING.md, CODE_OF_CONDUCT.md files

## v0.2.1 (2021-01-19)

### Features

- Add 'dual' action commands. These action commands publish position setpoints with a limited (truncated) maximum velocity.

## v0.2.0 (2020-12-16)

### Features

- **(Breaking Change)** Add support for YAML parameters and waypoint error check
- **(Breaking Change)** Remove explicit definition of number of commands in *.txt files
- Migrate flight setup parameters to *.yaml files
- Update README instructions including the latest changes
- Change coordinate frame from FRAME_BODY_NED to FRAME_LOCAL_NED
- Update format of position and velocity commands using 'pos' or 'vel'
to improve readability.

## v0.1.1 (2020-12-14)

### Features

- Include 'config_yaml' as roslaunch writable arg
- Add Changelog
- Add sequence to initialise realsense t265
- Add SLAM_wpts_lateral_no_yaw.txt file for testing SLAM wo yawing

### Fixes

- Remove unused dependencies
- Fix build errors
- Rename global variables
- Fix typos of px4_config.yaml

## v0.1.0 (2020-06-20)

Initial release.
