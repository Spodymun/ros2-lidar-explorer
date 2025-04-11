#!/bin/bash

# Check if an IP address is provided as the first argument
if [ -z "$1" ]; then
    echo "Missing IP address! Usage: ./startup.bash <IP>"
    exit 1
fi

# Assign the provided IP address to ESP_IP
ESP_IP=$1
echo "Using IP address: $ESP_IP"

# Prompt for the map name
read -p "🗺️  What's the map called? " MAP_NAME
echo "Using map name: $MAP_NAME"

# Reset previous SLAM and navigation state by removing relevant files
echo "Resetting previous SLAM and navigation state..."
rm -f ~/.ros/pose_db.db
rm -f ~/.ros/slam_toolbox*db
rm -f ~/.ros/seen_frontiers*
rm -f ~/.ros/*.yaml
rm -f ~/.ros/*.pgm

# Cleanup function to terminate all background processes when the script exits
cleanup() {
    echo "Terminating all background processes..."
    kill $bg_pid1 $bg_pid2 $bg_pid3 $bg_pid4 $bg_pid5 $bg_pid6 \
         $bg_pid7 $bg_pid8 $bg_pid9 $bg_pid10 $bg_pid11
}

# Set the cleanup function to run when the script exits
trap cleanup EXIT

# Start ROS2 processes with the provided IP

# Launch the robot with the provided IP address
ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!

# Launch the SLLidar driver
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py &
bg_pid2=$!

# Wait until the /scan_raw topic is available
until ros2 topic echo /scan_raw --once; do
  echo "⏳ Waiting for /scan_raw..."
  sleep 1
done
echo "✅ /scan_raw available – starting Relay..."

# Change directory to the Python scripts folder and run the scan timestamp relay
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 scan_timestamp_relay.py &
bg_pid10=$!

# Wait until the /scan topic is available
until ros2 topic echo /scan --once; do
  echo "Waiting for /scan..."
  sleep 1
done
echo "✅ /scan available."

# Launch the SLAM toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
bg_pid3=$!

# Launch the teleop_twist_keyboard for manual control in a new terminal
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; exec bash" &
bg_pid4=$!

# Run the ESP HTTP control script with the provided IP address
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 esp_http_control.py "$ESP_IP" &
bg_pid5=$!

# Open RViz for visualization in a new terminal
gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/explore.rviz; exec bash" &
bg_pid6=$!

# Launch the navigation system
ros2 launch nav2_bringup navigation_launch.py \
  autostart:=true \
  use_lifecycle_mgr:=true \
  params_file:=/home/robi/ws_lidar/src/ros2-lidar-explorer/config/nav2_params.yaml &
bg_pid7=$!

# Run the navigate relay script
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 navigate_relay.py &
bg_pid11=$!

# Launch the twist_mux for robot control
ros2 launch ros2-lidar-explorer twist_mux_launch.py &
bg_pid8=$!

# Relaunch the robot with the provided map name
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 relaunch.py "$MAP_NAME" &
bg_pid9=$!

# Wait to prevent the script from exiting immediately
wait