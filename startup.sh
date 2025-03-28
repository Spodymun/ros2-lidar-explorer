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
    kill $bg_pid7
    kill $bg_pid8
    kill $bg_pid9
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
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; exec bash" &
bg_pid4=$!
echo "Process 4 (teleop_twist_keyboard) started in a new terminal, PID: $bg_pid4"

# 5. Start the ESP control script with the provided IP
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 esp_http_control.py "$ESP_IP" & 
bg_pid5=$!
echo "Process 5 (esp_http_control.py) started in the background with IP $ESP_IP, PID: $bg_pid5"

# 6. Start RVIZ2 in a new terminal for visualization # -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/explore.rviz
gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/basic.rviz; exec bash" &
bg_pid6=$!
echo "Process 6 (rviz2) started in a new terminal, PID: $bg_pid6"

# 7. Start Nav2 for navigation
ros2 launch nav2_bringup navigation_launch.py use_sime_time:=false &
bg_pid7=$!
echo "Process 7 (navigation_launch.py started in the background), PID: $bg_pid7"

# 8. Start Twist_mux for input controll
ros2 launch ros2-lidar-explorer twist_mux_launch.py &
bg_pid8=$!
echo "Process 8 (twist_mux_launch), PID: $bg_pid8"

# 5. Start the ESP control script with the provided IP
# cd ~/ws_lidar/src/ros2-lidar-explorer/python
# python3 goal_reacher.py "$ESP_IP" & 
# bg_pid9=$!
# echo "Process 9 (esp_http_control.py) started in the background with IP $ESP_IP, PID: $bg_pid9"

# 9. Start the ESP control script with the provided IP
# cd ~/ws_lidar/src/m-explore-ros2/map_merge/launch
# python3 map_merge.launch.py "$ESP_IP" & 
# bg_pid9=$!
# echo "Process 9 (map_merge.launch.py) started in the background, PID: $bg_pid9"

# Wait to prevent the script from exiting immediately
wait