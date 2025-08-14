#!/bin/bash

if [ -z "$1" ]; then
    echo "❌ Missing IP address! Usage: ./servo_lidar.sh <IP>"
    exit 1
fi

ESP_IP=$1
echo "📡 Using IP address: $ESP_IP"

# Clean up old ROS artifacts
rm -f ~/.ros/pose_db.db ~/.ros/slam_toolbox*db ~/.ros/seen_frontiers* ~/.ros/*.yaml ~/.ros/*.pgm

# Servo VENV + ROS 2 Setup
source /opt/ros/jazzy/setup.bash
source ~/ws_lidar/install/setup.bash
source ~/ws_lidar/src/STServo_Python/venv-servo/bin/activate

# Graceful shutdown
cleanup() {
    echo "🛑 Shutting down background processes..."
    kill $bg_pid1 $bg_pid2 $bg_pid3 $bg_pid4 $bg_pid5 $bg_pid6 \
         $bg_pid7 $bg_pid8 $bg_pid9 $bg_pid10 $bg_pid11 $bg_pid12 2>/dev/null
}
trap cleanup EXIT

# Launch robot bringup
ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!
sleep 3

# Launch LIDAR
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py &
bg_pid2=$!

# Wait for /scan to be ready
until ros2 topic echo /scan --once; do
  echo "⏳ Waiting for /scan..."
  sleep 1
done
echo "✅ /scan detected"

# Start SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
bg_pid3=$!
sleep 2

# 🧭 NEW: Launch Navigation2
ros2 launch nav2_bringup navigation_launch.py \
  autostart:=true \
  use_lifecycle_mgr:=true \
  params_file:=/home/robi/ws_lidar/src/ros2-lidar-explorer/config/nav2_params.yaml &
bg_pid7=$!
sleep 3

# Open RViz
gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/3d.rviz; exec bash" &
bg_pid6=$!

# Twist mux
ros2 launch ros2-lidar-explorer twist_mux_launch.py &
bg_pid8=$!

# Teleop keyboard
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; exec bash" &
bg_pid4=$!

# ESP control script
python3 ~/ws_lidar/src/ros2-lidar-explorer/python/esp_http_control.py "$ESP_IP" &
bg_pid5=$!

# 🧩 Start Servo Node
python3 ~/ws_lidar/src/ros2-lidar-explorer/python/servo.py &
bg_pid9=$!
echo "⚙️  Servo Node gestartet"

# 🧩 NEW: Start Scan-to-PointCloud Node
python3 ~/ws_lidar/src/ros2-lidar-explorer/python/scan_to_pointcloud.py &
bg_pid12=$!
echo "🌐 PointCloud Node gestartet (/servo_lidar/pointcloud)"

# Done
wait
