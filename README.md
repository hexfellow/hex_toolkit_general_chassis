
# **hex_toolkit_general_chassis**

## **Overview**

The **hex_toolkit_general_chassis** package provides a suite of tools for robot chassis control, including keyboard and gamepad-based teleoperation.

### **Maintainer**

**Dong Zhaorui**: [847235539@qq.com](mailto:847235539@qq.com)

### **Verified Platforms**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**
- [ ] **Rockchip RK3588**

### **Verified ROS Versions**

- [x] **Noetic**
- [x] **Humble**
- [ ] **Jazzy**

---

## **Getting Started**

### **Dependencies**

For **Hex Chassis** users, we highly recommend using this package within our **Hex Docker Images** to ensure compatibility and streamlined setup.

If you prefer manual setup, please ensure the following dependencies are installed:

1. **ROS**  
   Refer to the official [ROS](http://wiki.ros.org/ROS/Installation) and [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) Installation Guide.

2. **pygame**  
   Install via pip:

   ```shell
   pip3 install pygame
   ```

3. **pynput**  
   Install via pip:

   ```shell
   pip3 install pynput
   ```

### **Installation**

1. Create a ROS workspace and navigate to the `src` directory:

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

### **Pre-Execution Steps**

1. Source the appropriate setup file for your ROS version:

   - For **ROS 1**:

     ```shell
     source devel/setup.bash --extend
     ```

   - For **ROS 2**:

     ```shell
     source install/setup.bash --extend
     ```

---

## **Demos**

- **Cart Gen**: Generate circular trajectories for testing.  
- **Joy Ctrl**: Control the chassis with a gamepad.  
- **Key Ctrl**: Control the chassis with a keyboard.

---

### **Cart Gen**

#### **Introduction**

The **Cart Gen** module generates a circular trajectory for chassis testing.

#### **Usage**

- For **ROS 1**:

  ```shell
  roslaunch hex_toolkit_general_chassis cart_gen.launch
  ```

- For **ROS 2**:

  ```shell
  ros2 launch hex_toolkit_general_chassis cart_gen.launch.py
  ```

#### **Published Topics**

| Topic Name     | Message Type                | Description                         |
| -------------- | --------------------------- | ----------------------------------- |
| `/target_pose` | `geometry_msgs/PoseStamped` | Publishes target poses for testing. |

#### **Parameters**

| Parameter Name      | Data Type        | Description                                      |
| ------------------- | ---------------- | ------------------------------------------------ |
| `rate_ros`          | `double`         | Execution rate of the ROS node (Hz).             |
| `model_base`        | `string`         | Frame ID of the chassis base.                    |
| `model_odom`        | `string`         | Frame ID of the odometry.                        |
| `cart_center`       | `vector<double>` | Coordinates of the trajectory's center (m).      |
| `cart_radius`       | `double`         | Radius of the trajectory (m).                    |
| `cart_period`       | `double`         | Time to complete one full circle (s).            |
| `cart_inverse_flag` | `bool`           | Specifies if the trajectory is counterclockwise. |

---

### **Joy Ctrl**

#### **Introduction**

The **Joy Ctrl** module allows users to control the chassis using a gamepad.

#### **Usage**

- For **ROS 1**:

  ```shell
  roslaunch hex_toolkit_general_chassis joy_ctrl.launch
  ```

- For **ROS 2**:

  ```shell
  ros2 launch hex_toolkit_general_chassis joy_ctrl.launch.py
  ```

#### **Published Topics**

| Topic Name     | Message Type                 | Description                                    |
| -------------- | ---------------------------- | ---------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes `twist` commands without timestamps. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes `twist` commands with timestamps.    |

#### **Parameters**

| Parameter Name | Data Type | Description                               |
| -------------- | --------- | ----------------------------------------- |
| `rate_ros`     | `double`  | Execution rate of the ROS node (Hz).      |
| `ratio_vx`     | `double`  | Ratio of x-axis linear velocity (m/s).    |
| `ratio_vy`     | `double`  | Ratio of y-axis linear velocity (m/s).    |
| `ratio_yaw`    | `double`  | Ratio of z-axis angular velocity (rad/s). |
| `joy_deadzone` | `double`  | Deadzone for joystick axis input.         |
| `joy_vx`       | `string`  | Axis name for x-axis linear velocity.     |
| `joy_vy`       | `string`  | Axis name for y-axis linear velocity.     |
| `joy_yaw`      | `string`  | Axis name for z-axis angular velocity.    |

---

### **Key Ctrl**

#### **Introduction**

The **Key Ctrl** module enables users to control the chassis using a keyboard.

#### **Usage**

- For **ROS 1**:

  ```shell
  roslaunch hex_toolkit_general_chassis key_ctrl.launch
  ```

- For **ROS 2**:

  ```shell
  ros2 launch hex_toolkit_general_chassis key_ctrl.launch.py
  ```

#### **Published Topics**

| Topic Name     | Message Type                 | Description                                    |
| -------------- | ---------------------------- | ---------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes `twist` commands without timestamps. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes `twist` commands with timestamps.    |

#### **Parameters**

| Parameter Name | Data Type        | Description                               |
| -------------- | ---------------- | ----------------------------------------- |
| `rate_ros`     | `double`         | Execution rate of the ROS node (Hz).      |
| `ratio_vx`     | `double`         | Ratio of x-axis linear velocity (m/s).    |
| `ratio_vy`     | `double`         | Ratio of y-axis linear velocity (m/s).    |
| `ratio_yaw`    | `double`         | Ratio of z-axis angular velocity (rad/s). |
| `key_vx`       | `vector<string>` | Key for x-axis linear velocity.           |
| `key_vy`       | `vector<string>` | Key for y-axis linear velocity.           |
| `key_yaw`      | `vector<string>` | Key for z-axis angular velocity.          |
