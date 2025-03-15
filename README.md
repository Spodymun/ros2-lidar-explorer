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
Some dependencies from **@Articulated Robotics' videos** are required.  
Please check the videos for details. 
(I'll hopefully update the list in the future) 

Run the following command to install the necessary ROS 2 packages:  
```bash
sudo apt update && sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-nav2-bringup 
```

#### ▶️ Running the Project  

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
