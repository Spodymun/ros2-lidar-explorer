#!/bin/bash

# Check if an IP address was provided as an argument
if [ -z "$1" ]; then
    echo "Missing IP address! Usage: ./startup.bash <IP>"
    exit 1
fi

ESP_IP=$1
echo "Using IP address: $ESP_IP"

# Cleanup function to terminate all background processes
cleanup() {
    echo "Terminating all background processes..."
    kill $bg_pid1
    kill $bg_pid2
    kill $bg_pid3
    kill $bg_pid4
    kill $bg_pid5
    kill $bg_pid6
}

# Ensure the cleanup function is executed when the script exits
trap cleanup EXIT

# Start ROS2 processes with the provided IP

# 1. Launch the robot with the given IP
ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!
echo "Process 1 (launch_robot.launch.py) started in the background, PID: $bg_pid1"

# 2. Start the RPLidar node
ros2 launch ros2-lidar-explorer rplidar.launch.py &
bg_pid2=$!
echo "Process 2 (rplidar.launch.py) started in the background, PID: $bg_pid2"

# 3. Start SLAM Toolbox for mapping
ros2 launch slam_toolbox online_async_launch.py &
bg_pid3=$!
echo "Process 3 (online_async_launch.py) started in the background, PID: $bg_pid3"

# 4. Start teleop_twist_keyboard in a new terminal for manual control
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash" &
bg_pid4=$!
echo "Process 4 (teleop_twist_keyboard) started in a new terminal, PID: $bg_pid4"

# 5. Start the ESP control script with the provided IP
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 esp_http_control.py "$ESP_IP" &
bg_pid5=$!
echo "Process 5 (esp_http_control.py) started in the background with IP $ESP_IP, PID: $bg_pid5"

# 6. Start RVIZ2 in a new terminal for visualization
gnome-terminal -- bash -c "rviz2; exec bash" &
bg_pid6=$!
echo "Process 6 (rviz2) started in a new terminal, PID: $bg_pid6"

# Wait to prevent the script from exiting immediately
wait