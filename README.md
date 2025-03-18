# ROS2 Lidar Mapping & Navigation  

## How This Works 🚀  

This project is based on **@Articulated Robotics' YouTube tutorials** ([Playlist](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)).  
I followed the series up to **"Using ros2_control to drive our robot (off the edge of the bench...)"** but have since made **many modifications**.  

Please review the **differences yourself**, as well as the **dependencies** used in the original videos.  
I may add a full dependency list soon (**hopefully**).  

---

## What Can You Do With This? 🤖  

By cloning this repository, you'll be able to:  
✅ Use **LiDAR A1/A2** for mapping  
✅ Create a **SLAM map** using `slam_toolbox`  

---

## Hardware Requirements 🔧  

- **UGV Platform:** Waveshare UGV02  
- **Compute:** Raspberry Pi 5 + Active Cooler  
  - Running **Ubuntu Noble 24.04**  
- **LiDAR Sensor:** A2M8 - R4  

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
Please check my code for any necessary IP address changes.
You'll need to set up a hotspot using a laptop, mobile phone, or another compatible device.
Once connected, you can find the IP address of your ESP.

#### 📦 Essential Packages  
Please check **@Articulated Robotics' videos** videos for details. 

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
    ros-jazzy-xacro    

sudo apt install -y python3-requests python3-urllib3 python3-numpy
```
# Matching Your System

## Matching Your Robot

### Measuring Wheel Radius & Wheel Distance
1. **`self.wheel_radius`**: Do not take real measurements, adjust it manually until the odometry matches the real-life movement.
2. **`self.wheel_offset_x`**: Do not take real measurements, adjust it manually until the odometry matches the real-life movement.

---

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
   
# ▶️ Running the Project  

Once everything is set up, follow these steps to launch mapping.  

```bash
cd ~/ws_lidar/src/ros2-lidar-explorer
./startup.sh IP_ADRESS_OF_YOUR_ESP
```
In Rviz you'll have to add laser_scan, map, robot and tf.
   - By display type laser_scan - by topic /scan
   - By display type map - by topic /map
   - By display type RobotModel
     - Description Topic /robot_description

Set your fixed frames to odom until the map is loaded, then set it to map and start exploring.
