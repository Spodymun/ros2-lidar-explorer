# ROS2 Lidar Mapping & Navigation  

## How This Works 🚀  

This project is based on **@Articulated Robotics' YouTube tutorials** ([Playlist](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)).  
I followed the series up to **"Using ros2_control to drive our robot (off the edge of the bench...)"** but have since made **many modifications**.  

## ⚠️ IMPORTANT

Please review the **differences yourself**, as well as the **dependencies** used in the original videos.  
I (**hopefully**) added the full list down below.

After cloning the other repositories, make sure to visit them yourself and work through their individual setup guides.

---

## What Can You Do With This? 🤖  

By cloning this repository, you'll be able to:  
✅ Use **LiDAR A1/A2** for mapping  
✅ Create a **SLAM map** using `slam_toolbox`  
✅ Navigates autonomously through the Room using **Nav2** and **m-explore-nav2**

---

## Hardware Requirements 🔧  

- **UGV Platform:** Waveshare UGV02  
- **Compute:** Raspberry Pi 5 + Active Cooler / Jetson Orin nano
  - Running **Ubuntu Noble (Pro) 24.04**  
- **LiDAR Sensor:** A2M8

I upgraded to a Jetson Orin Nano, so I no longer need some of my real-time settings. However, I’ve documented how to use them and everything you need to set them up.

---

## Setup Guide 🏗  

### 1️⃣ Install ROS 2 (Verified: **03.03.2025**)  
Follow the official ROS 2 **Jazzy** installation guide:  
🔗 [ROS 2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)  

### 2️⃣ Create a ROS2 Workspace (Verified: **14.03.2025**)  
Follow this guide to set up a workspace:  
🔗 [Creating a ROS 2 Workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)  

#### Tips:  
- My workspace is named **`ws_lidar`** – I recommend using the same name to avoid issues with my scripts.  
- Clone this repo into your workspace:  
  ```bash
  cd ~/ws_lidar/src
  git clone https://github.com/Spodymun/ros2-lidar-explorer
  ```
#### 🔁 Automatically Source ROS and Workspace on Every Shell Start

To avoid manually sourcing after each new terminal session or build, add the following lines to your `~/.bashrc`:

```bash
grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
grep -qxF 'source ~/ws_lidar/install/setup.bash' ~/.bashrc || echo 'source ~/ws_lidar/install/setup.bash' >> ~/.bashrc
```

#### 📦 Essential Packages  
Run the following command to install the necessary ROS 2 packages:

```bash
sudo apt update && sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rclpy \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-ament-index-python \
    ros-jazzy-xacro \
    ros-jazzy-navigation2 \
    ros-jazzy-twist-mux \
    ros-jazzy-ros2-control \
    ros-jazzy-nav2-bringup

sudo apt install -y python3-requests python3-urllib3 python3-numpy
```
---

### 🛠️ Setting Up RPLidar with ROS 2

This guide explains how to clone and configure the `sllidar_ros2` package to work with your RPLidar device in a ROS 2 workspace.

#### 📦 1. Clone the `sllidar_ros2` Driver

Navigate to your workspace’s `src` directory and clone the official ROS 2 driver from Slamtec:

```bash
cd ~/ws_lidar/src
git clone https://github.com/Slamtec/sllidar_ros2
```

#### ⚙️ 2. Configure the Driver

After cloning, navigate to the launch folder:

```bash
cd ~/ws_lidar/src/sllidar_ros2/launch
ls
```

Find the launch file that matches your RPLidar model.  
For example, if you're using an **A2M8**, it will likely be:

```bash
sllidar_a2m8_launch.py
```

Then make the following manual adjustments in that file:

#### ✅ Set the `scan_mode`

- Add or update the parameter `scan_mode` to `"Standard"`
- You can use another supported mode if your LIDAR model requires it

#### ✅ Update `frame_id`

- Replace all instances of `laser` with `laser_frame`
- This ensures TF compatibility across your system

#### ✅ Update the launch configuration

> **⚠️ WARNING**  
> You’ll only need the following step if you run into issues with the time frames:  
> 1. Copy `scan_timestamp_relay.py` from the `archives/` folder into the `python/` folder.  
> 2. Uncomment its launch command in `mapping.sh` (it’s currently commented out).  

Make sure your node is defined like this in your launch file:

```python
Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar_node',
    parameters=[{
        'channel_type': channel_type,
        'serial_port': serial_port,
        'serial_baudrate': serial_baudrate,
        'frame_id': 'laser_frame',
        'inverted': inverted,
        'angle_compensate': angle_compensate,
        'scan_mode': 'Standard'
    }],
    remappings=[
        ('/scan', '/scan_raw')  # Remap original scan topic
    ],
    output='screen'
)
``` 

#### ✍️ 3. Edit the Mapping File

If you’re using a different model than the A2M8, make sure to update your mapping script accordingly:

```bash
cd ~/ws_lidar/src/ros2-lidar-explorer
sudo nano mapping.sh
```

Look for this line:

```bash
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py
```

Change `sllidar_a2m8_launch.py` to the appropriate launch file for your Lidar model.

#### 🚀 4. Rebuild and Source the Workspace

Once your changes are made, rebuild your ROS 2 workspace:

```bash
cd ~/ws_lidar
colcon build
source install/setup.bash
```

> 💡 If `colcon build` throws an error the first time, just try it again.  
> Sometimes it resolves itself after a clean rebuild.

> **⚠️**
> If you run into problems with the LiDAR because it won’t activate, try:
>
> ```bash
> sudo chmod 777 /dev/ttyUSB0
> ```

Now your RPLidar should be ready to run with the correct configuration! 🎉

---

### 🧠 Autonomic Driving Integration for ROS 2 (Jazzy)

First, clone the `m-explore-ros2` exploration package into your workspace:

```bash
cd ~/ws_lidar/src
git clone https://github.com/robo-friends/m-explore-ros2.git
```
#### 🛠 Fixing Compatibility with ROS 2 Jazzy

ROS 2 Jazzy introduced a breaking change in the `tf2_geometry_msgs` package.  
The header file `tf2_geometry_msgs.h` has been renamed to `tf2_geometry_msgs.hpp`.  
To update all source files accordingly, run:

```bash
find ~/ws_lidar/src -type f \( -name "*.cpp" -or -name "*.h" -or -name "*.hpp" \) | xargs sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs.h|tf2_geometry_msgs/tf2_geometry_msgs.hpp|g'
```
#### 🧩 Updating Timer Callbacks

In **ROS 2 Jazzy**, the method `execute_callback()` from `rclcpp::TimerBase` has changed.  
It now requires an **explicit argument** of type `std::shared_ptr<void>`.  
To maintain compatibility, update all direct calls in one go:

```bash
find ~/ws_lidar/src -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -print0 \
  | xargs -0 sed -i -E 's/execute_callback *\(\s*\)/execute_callback(nullptr)/g'
```
#### 🐢 Slower robot = more patience

Since my robot moves a bit slower, I had to give it more time to plan and make progress.  
To do this, I adjusted the following parameters in:

```bash
m-explore-ros2/explore/config/params.yaml
```
```yaml
planner_frequency: 0.1
progress_timeout: 40.0
```
Once the changes are complete, build your ROS 2 workspace to apply them:

```bash
cd ~/ws_lidar
colcon build --symlink-install
```

> ⚠️ **Warning**
>
> If this crashes due to OpenCV issues **and** OpenCV is already installed,
> try the following *(happened to me after switching to Jetson)*:
>
> ## 🛠️ Step 1: Create the Bash Script
> Save the following script as `link_opencv_libs.bash`:
> ```bash
> for f in /opt/opencv-4.8.0/lib/libopencv_*.so.4.8.0; do
>   sudo ln -sf "$f" /usr/lib/$(basename "$f")
> done
> ```
>
> ## ▶️ Step 2: Make the Script Executable
> ```bash
> chmod +x link_opencv_libs.bash
> ```
>
> ## 🚀 Step 3: Run the Script
> ```bash
> ./link_opencv_libs.bash
> ```
>
> ## 🧹 Step 4: Clean CMake and ROS Build Directories
> Make sure you are back in your Workspace
> ```bash
> rm -rf build/ install/ log/
> ```
>
> ## 🔁 Step 5: Rebuild ROS 2 Package
> ```bash
> colcon build
> ```
>
> ## ➕ Additionally:
> ```bash
> grep -qxF 'export LD_LIBRARY_PATH=/opt/opencv-4.8.0/lib:$LD_LIBRARY_PATH' ~/.bashrc || echo 'export LD_LIBRARY_PATH=/opt/opencv-4.8.0/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
> ```
---

## 🛠️ Troubleshooting: TF Transformation Delays & Nav2 Shutdowns

I ran into a persistent issue where my navigation tool (Nav2) kept shutting down without any obvious errors. After **many hours of debugging**, I finally discovered the root cause:  
➡️ **The TF transformations were too slow**, which caused Nav2 to stop working due to missing or outdated "live" data.

### ✅ My Fix: Real-Time Data Optimization

To solve this, I took several steps:

1. **Upgraded to Ubuntu Pro** ([Get it here – it's free](https://ubuntu.com/pro/subscribe))  
   Ubuntu Pro allowed me to access better real-time processing capabilities, which significantly **improved the speed of TF frame transformations**.

> **⚠️ WARNING**  
> This is not available for the Jetson because Ubuntu 24.04 is **not supported** by Nvidia.

2. **Increased TF tolerances in Nav2**  
   I changed **all transform tolerances** in the `nav2_params.yaml` config file to `1.0`.  
   You can find this configuration in:  
   ```yaml
   config/nav2_params.yaml
   ```
3. **Adjusted scan data publishing timing**  
   I added a custom script that adjusts the timestamp of the scan data.  
   - Original data is published on: `/scan_raw`  
   - The script updates the time frame and republishes it on: `/scan`  
   - 📄 The script can be found here:  
     ```python
     python/scan_timestamp_relay.py
     ```
> **⚠️ WARNING**  
> You'll need to add it by yourself

> ⚙️ If you cloned this repo, steps 2 (and 3) are **already included**.

### 🔄 Still Having Issues?

If you're still running into unexplained Nav2 crashes or TF timeout issues, I **highly recommend upgrading to Ubuntu Pro** and experimenting with:
- TF tolerances in the Nav2 config
- Timing of your sensor data publication

---
# RealSense D415 + ROS 2 Jazzy + RViz2 Integration

This guide shows how to get a **working RGB image + pointcloud stream** from your Intel® RealSense™ D415 in **RViz2** 

## 🔧 Step-by-Step Setup

### 1. Install librealsense

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.56.1  # or higher

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 2. Prepare ROS 2 workspace

```bash
cd ~/ws_lidar/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
```
### 3. Install dependencies

```bash
cd ~/ws_lidar
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```
### 4. Fix build issue

> The `rotation_filter` is no longer supported in librealsense ≥ 2.56. You must comment it out.

```bash
sudo nano ~/ws_lidar/src/realsense-ros/realsense2_camera/src/base_realsense_node.cpp
```

- Go to line **232** and comment out:
  ```cpp
  // filters.push_back(std::make_shared<NamedFilter>(std::make_shared<rs2::rotation_filter>(...)));
  ```

Save and close.

### 5. Build the workspace

```bash
cd ~/ws_lidar
colcon build --symlink-install
source install/setup.bash
```
---

# Matching Your System

## Matching Your Robot

### Measuring Wheel Radius & Wheel Distance
1. **`self.wheel_radius`**: Do not take real measurements, adjust it manually until the odometry matches the real-life movement.
2. **`self.wheel_offset_x`**: Do not take real measurements, adjust it manually until the odometry matches the real-life movement.

## Encoder Ticks & Motion Calibration

### 1. Measuring Encoder Ticks per Revolution
1. **Prepare the wheel**: Ensure it is free to rotate.
2. **Read the initial value**: Note the encoder reading.
3. **Rotate the wheel**: Complete one full revolution.
4. **Read the new value**: Calculate the difference:

   ```python
   TICKS_PER_REV = encoder_after - encoder_before
   print(f"Ticks per revolution: {TICKS_PER_REV}")
   ```
### 2. Calibrating Straight-Line Motion
1. **Perform the test**: Use `move.py` to move the robot forward.
2. **Measure the distance**:  
   - Observe the actual distance the robot moves.  
   - Compare it with the distance shown in the `/odom` data in RViz.  
3. **Adjust the scaling factor in `esp_http_control.py`**:
   
   ```python
   scaling_factor_straight = rviz_distance / actual_distance
   ```
   If the robot moves too short, increase the factor. If it moves too far, decrease it.

### 3. Calibrating Rotational Motion
1. **Perform the test**: Use `move.py` to rotate the robot in place.
2. **Measure the rotation angle**:  
   - Check how much the robot has turned in reality.  
   - Compare this with the rotation angle shown in the `/odom` data in RViz.  
3. **Adjust the scaling factor in `esp_http_control.py`**:  

   ```python
   scaling_factor_circle = rviz_angle / actual_angle
   ```
   If the rotation is too small, increase the factor. If it's too large, decrease it.

---
   
# ▶️ Running the Project  
You'll need to set up a hotspot using a laptop, smartphone, or another compatible device.
Once the device is connected to the hotspot, you can locate the IP address of your ESP.

> ⚠️ It's important that the Raspberry Pi and the ESP are connected to the same Wi-Fi network — either via a shared hotspot or by directly connecting to each other's Wi-Fi.

Once everything is set up, follow these steps to launch mapping.  

> ⚠️ **Make sure you deactivate the camera launch if your system can't provide enough GPU acceleration**  

```bash
cd ~/ws_lidar/src/ros2-lidar-explorer
./mapping.sh IP_ADRESS_OF_YOUR_ESP
```

