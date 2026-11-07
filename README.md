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


## Position feedback (AX-12A potentiometer)

In wheel mode the AX-12A still reports its potentiometer position (register 36, 0..1023 over 300°;
the remaining 60° is a dead zone with garbage readings). By default the driver reads it every cycle,
unwraps it into a continuous wheel angle and publishes real `position` / `velocity` state — before,
these were just the integrated command.

* Dead zone handling: readings are accepted only if they are consistent with the commanded motion;
  inside the dead zone the position is dead-reckoned and re-synchronised at the next valid reading
  (see `position_unwrapper.hpp`, covered by unit tests).
* Present-speed register (38) is **not** used: it returned garbage in wheel mode.
* `on_activate` now fails (instead of silently succeeding) if the port, a servo, or the first position
  read does not work, and `read()` returns an error after 10 consecutive failed reads (~0.3 s at 30 Hz).
* Optional hardware parameter `position_feedback` (default `true`); set `false` to get the old
  open-loop behaviour.
* Requests below the minimum wheel speed (80/1023 units, ~0.49 rad/s) are zeroed by the driver; a warning
  is logged when this happens.

### Required: FTDI latency timer

Each register read takes 8–16 ms with the Linux default `latency_timer` of 16 ms, which is too slow for two
servos at 30 Hz. Set it to 1 ms:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# permanent:
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
  | sudo tee /etc/udev/rules.d/99-ftdi-latency.rules
```

Run the unit tests with `colcon test --packages-select dynamixel_hw`.

## Safety

* **Supply voltage monitoring.** About once per second the driver reads each servo's supply voltage
  (register 42). It logs a warning below `min_voltage_warn` (default 10.5 V) and, after 3 consecutive
  readings below `min_voltage_stop` (default 9.6 V), stops the wheels and reports an error.
  The defaults suit a 3S Li-ion/LiPo pack (AX-12A minimum is 9 V). Set a threshold to `0` to disable it.
* **Stop on error.** `on_error` sets both servo speeds to 0 and disables torque. A servo keeps its last
  speed command when the controlling computer disappears (crash, reset, power loss), and the driver
  cannot prevent that. For a robot that drives on the floor, add a hardware cut-off that defaults to off.
* Velocity loop parameters: `velocity_loop` (default `true`), `velocity_kp` (0.3), `velocity_ki` (2.0).
