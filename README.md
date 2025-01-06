# hex_toolkit_general_chassis

## Overview

This package provides general tools for robot chassis, including keyboard-based teleoperation.

### Maintainer

[Dong Zhaorui](mailto:847235539@qq.com)

### Prerequisites

*For Hex Chassis users, we strongly recommend using this package within our **Hex Docker Images**.*

#### Dependencies

1. **ROS**  
   Follow the [ROS Installation Guide](http://wiki.ros.org/ROS/Installation).

2. **pygame**  

   Install using pip:

   ```shell
   pip3 install pygame
   ```

3. **pynput**  

   Install using pip:

   ```shell
   pip3 install pynput
   ```

#### Supported Platforms

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**
- [ ] **Rockchip RK3588**

#### Verified ROS Versions

- [x] **Noetic**
- [x] **Humble**
- [ ] **Jazzy**

---

## Getting Started

### Installation

1. Create a ROS workspace named `catkin_ws` and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Build the workspace:

   - For **ROS 1**:

     ```shell
     cd ../
     catkin_make
     ```

   - For **ROS 2**:

     ```shell
     cd ../
     colcon build
     ```

### Pre-Execution Steps

1. Source the appropriate setup file:

   - For **ROS 1**:

     ```shell
     source devel/setup.bash --extend
     ```

   - For **ROS 2**:

     ```shell
     source install/setup.bash --extend
     ```

---

## Demos

### Available Demos

- Cart Gen
- Joy Ctrl
- Key Ctrl

### Cart Gen

#### Introduction

The Cart Gen module generates a circular trajectory for test.

#### Usage

- For **ROS 1**:

  ```shell
  roslaunch hex_toolkit_general_chassis cart_gen.launch
  ```

- For **ROS 2**:

  ```shell
  ros2 launch hex_toolkit_general_chassis cart_gen.launch.py
  ```

#### Published Topics

| Topic Name     | Message Type                | Description                             |
| -------------- | --------------------------- | --------------------------------------- |
| `/target_pose` | `geometry_msgs/PoseStamped` | Publishes the target poses for testing. |

#### Subscribed Topics

None

#### Parameters

| Parameter Name      | Data Type        | Description                                     |
| ------------------- | ---------------- | ----------------------------------------------- |
| `rate_ros`          | `double`         | Execution rate of the ROS node (Hz).            |
| `model_base`        | `string`         | Frame ID of the chassis base.                   |
| `model_odom`        | `string`         | Frame ID of the odometry.                       |
| `cart_center`       | `vector<double>` | Coordinates of the trajectory's center (m).     |
| `cart_radius`       | `double`         | Radius of the trajactory (m).                   |
| `cart_period`       | `double`         | Time taken to complete one full circle (s).     |
| `cart_inverse_flag` | `bool`           | Determines if the trajectory is anti-clockwise. |

### Joy Ctrl

#### Introduction

The Joy Ctrl module publish `twist` cmd according to gamepad input.

#### Usage

- For **ROS 1**:

  ```shell
  roslaunch hex_toolkit_general_chassis joy_ctrl.launch
  ```

- For **ROS 2**:

  ```shell
  ros2 launch hex_toolkit_general_chassis joy_ctrl.launch.py
  ```

#### Published Topics

| Topic Name     | Message Type                 | Description                                 |
| -------------- | ---------------------------- | ------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes the twist cmd without time stamp. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes the twist cmd with time stamp.    |

#### Subscribed Topics

None

#### Parameters

| Parameter Name | Data Type | Description                               |
| -------------- | --------- | ----------------------------------------- |
| `rate_ros`     | `double`  | Execution rate of the ROS node (Hz).      |
| `ratio_vx`     | `double`  | Ratio of x-axis linear velocity (m/s).    |
| `ratio_vy`     | `double`  | Ratio of y-axis linear velocity (m/s).    |
| `ratio_yaw`    | `double`  | Ratio of z-axis angular velocity (rad/s). |
| `joy_deadzone` | `double`  | Deadzone of joy axis.                     |
| `joy_vx`       | `string`  | Axis name of x-axis linear velocity.      |
| `joy_vy`       | `string`  | Axis name of y-axis linear velocity.      |
| `joy_yaw`      | `string`  | Axis name of z-axis angular velocity.     |

### Key Ctrl

#### Introduction

The Key Ctrl module publish `twist` cmd according to keyboard input.

#### Usage

- For **ROS 1**:

  ```shell
  roslaunch hex_toolkit_general_chassis key_ctrl.launch
  ```

- For **ROS 2**:

  ```shell
  ros2 launch hex_toolkit_general_chassis key_ctrl.launch.py
  ```

#### Published Topics

| Topic Name     | Message Type                 | Description                                 |
| -------------- | ---------------------------- | ------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes the twist cmd without time stamp. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes the twist cmd with time stamp.    |

#### Subscribed Topics

None

#### Parameters

| Parameter Name | Data Type        | Description                               |
| -------------- | ---------------- | ----------------------------------------- |
| `rate_ros`     | `double`         | Execution rate of the ROS node (Hz).      |
| `ratio_vx`     | `double`         | Ratio of x-axis linear velocity (m/s).    |
| `ratio_vy`     | `double`         | Ratio of y-axis linear velocity (m/s).    |
| `ratio_yaw`    | `double`         | Ratio of z-axis angular velocity (rad/s). |
| `key_vx`       | `vector<string>` | Key name of x-axis linear velocity.       |
| `key_vy`       | `vector<string>` | Key name of y-axis linear velocity.       |
| `key_yaw`      | `vector<string>` | Key name of z-axis angular velocity.      |
