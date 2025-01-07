
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

For **Hex Chassis** users, we highly recommend using this package within our **Hex Docker Images** to ensure compatibility and an optimized setup experience.

If you choose to set up manually, please install the following dependencies:

1. **ROS/ROS2**  
   Follow the official [ROS Installation Guide](http://wiki.ros.org/ROS/Installation) or the [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

2. **pygame**  
   Install via pip:

   ```bash
   pip3 install pygame
   ```

3. **pynput**  
   Install via pip:

   ```bash
   pip3 install pynput
   ```

### **Installation**

1. Create a ROS workspace and navigate to the `src` directory:

   ```bash
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone the required repositories:

   ```bash
   git clone https://github.com/hexfellow/hex_toolkit_general_chassis.git
   ```

3. Build the workspace:

   - **ROS 1**:

     ```bash
     cd ../
     catkin_make
     ```

   - **ROS 2**:

     ```bash
     cd ../
     colcon build
     ```

### **Pre-Execution Steps**

Source the appropriate setup file based on your ROS version:

- **ROS 1**:

  ```bash
  source devel/setup.bash --extend
  ```

- **ROS 2**:

  ```bash
  source install/setup.bash --extend
  ```

---

## **Nodes**

The package provides the following nodes:

- **Cart Gen**: Generates circular trajectories for testing.  
- **Joy Ctrl**: Enables chassis control using a gamepad.  
- **Key Ctrl**: Enables chassis control using a keyboard.

---

### **Cart Gen**

#### **Introduction**

The **Cart Gen** node generates a circular trajectory for testing purposes.

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

The **Joy Ctrl** node allows users to control the chassis using a gamepad.

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
| `joy_vx`       | `string`  | Axis for x-axis linear velocity.          |
| `joy_vy`       | `string`  | Axis for y-axis linear velocity.          |
| `joy_yaw`      | `string`  | Axis for z-axis angular velocity.         |

---

### **Key Ctrl**

#### **Introduction**

The **Key Ctrl** node allows users to control the chassis using a keyboard.

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
| `key_vx`       | `vector<string>` | Keys for x-axis linear velocity.          |
| `key_vy`       | `vector<string>` | Keys for y-axis linear velocity.          |
| `key_yaw`      | `vector<string>` | Keys for z-axis angular velocity.         |

---

## **Launch Files**

The package provides the following launch files:

- **Cart Gen**: Generates circular trajectories for testing.  
- **Joy Ctrl**: Enables chassis control using a gamepad.  
- **Key Ctrl**: Enables chassis control using a keyboard.

---

### **Cart Gen**

#### **Introduction**

This launch file launches `cart_gen.py`.

#### **Usage**

- **ROS 1**:

  ```bash
  roslaunch hex_toolkit_general_chassis cart_gen.launch
  ```

- **ROS 2**:

  ```bash
  ros2 launch hex_toolkit_general_chassis cart_gen.launch.py
  ```

---

### **Joy Ctrl**

#### **Introduction**

This launch file launches `joy_ctrl.py`.

#### **Usage**

- **ROS 1**:

  ```bash
  roslaunch hex_toolkit_general_chassis joy_ctrl.launch
  ```

- **ROS 2**:

  ```bash
  ros2 launch hex_toolkit_general_chassis joy_ctrl.launch.py
  ```

---

### **Key Ctrl**

#### **Introduction**

This launch file launches `key_ctrl.py`.

#### **Usage**

- **ROS 1**:

  ```bash
  roslaunch hex_toolkit_general_chassis key_ctrl.launch
  ```

- **ROS 2**:

  ```bash
  ros2 launch hex_toolkit_general_chassis key_ctrl.launch.py
  ```
