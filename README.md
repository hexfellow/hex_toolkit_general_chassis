
# **hex_toolkit_general_chassis**

## **Overview**

The **hex_toolkit_general_chassis** package provides a suite of tools for robot chassis control, including keyboard-based teleoperation.

### **Maintainer**

**Dong Zhaorui**: [847235539@qq.com](mailto:847235539@qq.com)

#### **Supported Platforms**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**
- [ ] **Rockchip RK3588**

#### **Verified ROS Versions**

- [x] **Noetic**
- [x] **Humble**
- [ ] **Jazzy**

---

## **Getting Started**

### **Dependencies**

For **Hex Chassis** users, we strongly recommend using this package within our **Hex Docker Images** to ensure compatibility and optimal performance.

For other users, you can install the dependencies manually

1. **ROS**  
   Follow the official ROS/ROS2 Installation Guide.

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

## **Demos**

### **Available Demos**

- **Cart Gen**
- **Joy Ctrl**
- **Key Ctrl**

---

### **Cart Gen**

#### **Introduction**

The **Cart Gen** module generates a circular trajectory for testing.

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

#### **Subscribed Topics**

None

#### **Parameters**

| Parameter Name      | Data Type        | Description                                       |
| ------------------- | ---------------- | ------------------------------------------------- |
| `rate_ros`          | `double`         | Execution rate of the ROS node (Hz).              |
| `model_base`        | `string`         | Frame ID of the chassis base.                     |
| `model_odom`        | `string`         | Frame ID of the odometry.                         |
| `cart_center`       | `vector<double>` | Coordinates of the trajectory's center (m).       |
| `cart_radius`       | `double`         | Radius of the trajectory (m).                     |
| `cart_period`       | `double`         | Time for a full circle (s).                       |
| `cart_inverse_flag` | `bool`           | Determines if the trajectory is counterclockwise. |

---

### **Joy Ctrl**

#### **Introduction**

The **Joy Ctrl** module publishes `twist` commands based on gamepad input.

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

| Topic Name     | Message Type                 | Description                                     |
| -------------- | ---------------------------- | ----------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes `twist` commands without a timestamp. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes `twist` commands with a timestamp.    |

#### **Subscribed Topics**

None

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

The **Key Ctrl** module publishes `twist` commands based on keyboard input.

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

| Topic Name     | Message Type                 | Description                                     |
| -------------- | ---------------------------- | ----------------------------------------------- |
| `/unsafe_ctrl` | `geometry_msgs/Twist`        | Publishes `twist` commands without a timestamp. |
| `/vel_ctrl`    | `geometry_msgs/TwistStamped` | Publishes `twist` commands with a timestamp.    |

#### **Subscribed Topics**

None

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
