# hex_toolkit_general_chasssis

## What does this package do

This package 包含了包括键盘控制在内的底盘通用工具.

## Maintainer

[Dong Zhaorui](mailto:847235539@qq.com)

## Prerequisites

***For Hex Chassis Users, we highly recommend you using this pkg in our docker images.***

### Dependencies

* **ROS**:  
   Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)

* **pygame**:

   ```shell
   pip3 install pygame
   ```
  
* **pynput**

   ```shell
   pip3 install pynput
   ```

### Verified Platforms

* [x] **x64**
* [ ] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

### Verified ROS Version

* [x] **Noetic**
* [x] **Humble**
* [ ] **Jazzy**

## Getting Started

### Installation

1. Create a workspace `catkin_ws` and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Navigate back to the `catkin_ws` directory and build the workspace:

   * For ROS 1:

   ```shell
   cd ../
   catkin_make
   ```

   * For ROS 2:

   ```shell
   cd ../
   colcon build
   ```

### Before Running

1. Source the `setup.bash` file and run the tests:

   * For ROS 1:

   ```shell
   source devel/setup.bash --extend
   ```

   * For ROS 2:

   ```shell
   source install/setup.bash --extend
   ```

## Demos

### Demo List

* Cart Gen
* Joy Ctrl
* Key Ctrl

### Cart Gen

#### Intro

发布一个圆形轨迹用于测试.

#### Usage

* For ROS 1:

```shell
roslaunch hex_toolkit_general_chassis cart_gen.launch
```

* For ROS 2:

```shell
ros2 launch hex_toolkit_general_chassis cart_gen.launch.py
```

#### Publisher

| Node           | Msg Type                        | Description                       |
| -------------- | ------------------------------- | --------------------------------- |
| `/target_pose` | `geometry_msgs.msg.PoseStamped` | Example of a command publication. |

#### Subscriber

None

#### Parameters

| Name                | Data Type     | Description                        |
| ------------------- | ------------- | ---------------------------------- |
| `rate`              | `int`         | Rate of the ROS node.              |
| `model_path`        | `string`      | Path to the URDF file.             |
| `model_joint_names` | `vec<string>` | Names of the joints.               |
| `limit_pos_low`     | `vec<double>` | Lower bound of the joint position. |
| `limit_pos_high`    | `vec<double>` | Upper bound of the joint position. |
| `limit_vel`         | `vec<double>` | Velocity limits of the joints.     |
| `limit_eff`         | `vec<double>` | Effort limits of the joints.       |