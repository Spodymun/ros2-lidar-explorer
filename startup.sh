#!/bin/bash

# Check if an IP address was provided as an argument
if [ -z "$1" ]; then
    echo "Missing IP address! Usage: ./startup.bash <IP>"
    exit 1
fi

ESP_IP=$1
echo "Using IP address: $ESP_IP"

# Optional: Reset SLAM + Nav2 state
echo "Resetting previous SLAM and navigation state..."
rm -f ~/.ros/pose_db.db
rm -f ~/.ros/slam_toolbox*db
rm -f ~/.ros/seen_frontiers*
rm -f ~/.ros/*.yaml
rm -f ~/.ros/*.pgm

# Cleanup function to terminate all background processes
cleanup() {
    echo "Terminating all background processes..."
    kill $bg_pid1 $bg_pid2 $bg_pid3 $bg_pid4 $bg_pid5 $bg_pid6 \
         $bg_pid7 $bg_pid8 $bg_pid9 $bg_pid10 $bg_pid11
}

# Ensure the cleanup function is executed when the script exits
trap cleanup EXIT

# Start ROS2 processes with the provided IP

# 1. Launch the robot with the given IP
ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!
echo "Process 1 (launch_robot.launch.py) started, PID: $bg_pid1"

# 2. Start the RPLidar node
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py &
bg_pid2=$!
echo "Process 2 (rplidar.launch.py) started, PID: $bg_pid2"

until ros2 topic echo /scan_raw --once; do
  echo "⏳ Warte auf /scan_raw..."
  sleep 1
done
echo "✅ /scan_raw verfügbar – starte Relay..."

# 2.5 Start scan timestamp relay
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 scan_timestamp_relay.py &
bg_pid10=$!
echo "Process 10 (scan_timestamp_relay.py) gestartet, PID: $bg_pid10"

until ros2 topic echo /scan --once; do
  echo "Warte auf /scan..."
  sleep 1
done
echo "✅ /scan verfügbar."

# 3. Start SLAM Toolbox for mapping
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
bg_pid3=$!
echo "Process 3 (slam_toolbox) started, PID: $bg_pid3"

# 4. Start teleop_twist_keyboard in a new terminal
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; exec bash" &
bg_pid4=$!
echo "Process 4 (teleop_twist_keyboard) started, PID: $bg_pid4"

# 5. Start ESP control script
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 esp_http_control.py "$ESP_IP" &
bg_pid5=$!
echo "Process 5 (esp_http_control.py) started, PID: $bg_pid5"

# 6. Start RVIZ2 in new terminal
gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/explore.rviz; exec bash" &
bg_pid6=$!
echo "Process 6 (rviz2) started, PID: $bg_pid6"

# 7. Start Nav2 for navigation
ros2 launch nav2_bringup navigation_launch.py \
  autostart:=true \
  use_lifecycle_mgr:=true \
  params_file:=/home/robi/ws_lidar/src/ros2-lidar-explorer/config/nav2_params.yaml &
bg_pid7=$!
echo "Process 7 (navigation_launch.py) started, PID: $bg_pid7"

# 7.5 Start navigate_relay.py node
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 navigate_relay.py &
bg_pid11=$!
echo "Process 11 (navigate_relay.py) gestartet, PID: $bg_pid11"

# 8. Start Twist_mux
ros2 launch ros2-lidar-explorer twist_mux_launch.py &
bg_pid8=$!
echo "Process 8 (twist_mux_launch) started, PID: $bg_pid8"

# 9. Start the ESP relaunch watchdog
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 relaunch.py &
bg_pid9=$!
echo "Process 9 (relaunch.py) started, PID: $bg_pid9"

# Wait to prevent the script from exiting
wait
