# ros2_control_dynamixel_ax12a Library

 ROS2 Control Hardware interface for Dynamixel AX-12A servos controlled by diff_drive_controller

## Usage

Clone Dynamixel SDK and this project into your workspace.

Example controllers config file:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 30

    diff_cont:
      type: diff_drive_controller/DiffDriveController

    joint_broad:
      type: joint_state_broadcaster/JointStateBroadcaster
  

diff_cont:
  ros__parameters:
    publish_rate: 30.0

    base_frame_id: base_link

    left_wheel_names: ['left_wheel_joint']
    right_wheel_names: ['right_wheel_joint']

    wheel_separation: 0.184
    wheel_radius: 0.06

    use_stamped_vel: false
```

Example xacro file:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <ros2_control name="DynamixelHw" type="system">
        <hardware>
            <plugin>dynamixel_hw/DynamixelHw</plugin>
            <param name="usb_port">/dev/ttyUSB0</param>
            <param name="baud_rate">1000000</param>
            <param name="left_wheel_name">left_wheel_joint</param>
            <param name="right_wheel_name">right_wheel_joint</param>
        </hardware>
        <joint name="left_wheel_joint">
            <param name="id">10</param>
            <param name="invert">false</param>
            <command_interface name="velocity">
                <param name="min">-6</param>
                <param name="max">6</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>
        <joint name="right_wheel_joint">
            <param name="id">11</param>
            <param name="invert">true</param>
            <command_interface name="velocity">
                <param name="min">-6</param>
                <param name="max">6</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>
    </ros2_control>
</robot>
```

## 🧱 Building the Project (ROS 2 Jazzy)

> **Prerequisites:**  
> Ubuntu 24.04 (or Raspberry Pi with Ubuntu 24.04)  
> ROS 2 **Jazzy Jalisco** installed and sourced from `/opt/ros/jazzy`.

---

### 1️⃣ Bootstrap dependencies (run once)
From the repository root:

```bash
bash scripts/bootstrap.sh
```

This script installs `ros-dev-tools`, initializes and updates `rosdep`, and installs all missing system dependencies so your packages can be built cleanly.

---

### 2️⃣ Build the workspace
After bootstrapping (or in a new shell):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

> 💡 The `--symlink-install` option creates symbolic links instead of copying files,  
> which allows you to edit source, launch, or configuration files without rebuilding.

---

### 3️⃣ Source the environment
After a successful build:

```bash
source install/setup.bash
```

Your hardware plugin (`dynamixel_hw`) and the vendor library (`dynamixel_sdk_vendor`)  
are now available to ROS 2 and can be used in other workspaces.

---

### 4️⃣ Using this build in conjunction with another workspace

If you have another ROS 2 workspace (for example, your robot’s launch or simulation repo),  
you can **overlay** this workspace by sourcing both in order:

```bash
# Source base ROS 2 installation
source /opt/ros/jazzy/setup.bash

# Source the Dynamixel control workspace (this repo)
source ~/repos/ros2_control_dynamixel_ax12a/install/setup.bash

# Now source your robot workspace
source ~/repos/bungy_robot/install/setup.bash
```

This ensures that your robot workspace sees and can load the `dynamixel_hw` plugin.

---

### 🔧 Clean rebuild (if needed)

```bash
colcon build --symlink-install --cmake-clean-cache
# or start completely fresh
rm -rf build install log && colcon build --symlink-install
```

---

### ✅ Optional sanity check

Verify that your plugin is visible to `hardware_interface`:

```bash
ros2 pkg plugins --package hardware_interface | grep dynamixel_hw
```

