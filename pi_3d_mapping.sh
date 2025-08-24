#!/bin/bash

# ---- Args & Setup ----
if [ -z "$1" ]; then
    echo "❌ Missing IP address! Usage: $0 <IP>"
    exit 1
fi

ESP_IP=$1
echo "📡 Using IP address: $ESP_IP"

read -p "Optional: 3D map filename to save on exit (empty = skip), e.g. /home/robi/my_octomap.bt : " SAVE_3D_NAME
[ -n "$SAVE_3D_NAME" ] && echo "💾 Will save OctoMap to: $SAVE_3D_NAME"

# ROS env
source /opt/ros/jazzy/setup.bash
source ~/ws_lidar/install/setup.bash

# Servo VENV
source ~/ws_lidar/src/STServo_Python/venv-servo/bin/activate

# Clean old artifacts
rm -f ~/.ros/pose_db.db ~/.ros/slam_toolbox*db ~/.ros/seen_frontiers* ~/.ros/*.yaml ~/.ros/*.pgm

# ---- Cleanup / saver ----
save_3d_map() {
  if [ -n "$SAVE_3D_NAME" ]; then
    echo "💾 Saving OctoMap to $SAVE_3D_NAME ..."
    # .bt (binär) oder .ot – Dateiendung bestimmt Format
    ros2 run octomap_server octomap_saver -f "$SAVE_3D_NAME"
  fi
}

cleanup() {
    echo "🛑 Shutting down background processes..."
    save_3d_map
    kill $bg_pid1 $bg_pid2 $bg_pid3 $bg_pid4 $bg_pid5 $bg_pid6 \
         $bg_pid7 $bg_pid8 $bg_pid9 $bg_pid10 $bg_pid11 $bg_pid12 \
         $bg_pid13 2>/dev/null
}
trap cleanup EXIT INT

# ---- Bringup Robot ----
ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!
sleep 3

# ---- Lidar Treiber ----
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py &
bg_pid2=$!

# Warten bis /scan steht
until ros2 topic echo /scan --once >/dev/null 2>&1; do
  echo "⏳ Waiting for /scan..."
  sleep 1
done
echo "✅ /scan detected"

# ---- SLAM (2D, weiter für AMCL/Nav2 nützlich) ----
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
bg_pid3=$!
sleep 2

# ---- Nav2 mit deinen Parametern (VoxelLayer + projected_map) ----
ros2 launch nav2_bringup navigation_launch.py \
  autostart:=true \
  use_lifecycle_mgr:=true \
  params_file:=/home/robi/ws_lidar/src/ros2-lidar-explorer/config/nav2_params_3d.yaml &
bg_pid4=$!
sleep 3

# ---- RViz (3D-Ansicht) ----
gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/3d.rviz; exec bash" &
bg_pid5=$!

# ---- Twist mux ----
ros2 launch ros2-lidar-explorer twist_mux_launch.py &
bg_pid6=$!

# ---- Teleop ----
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; exec bash" &
bg_pid7=$!

# ---- ESP ----
python3 ~/ws_lidar/src/ros2-lidar-explorer/python/esp_http_control.py "$ESP_IP" &
bg_pid8=$!

# ---- Servo (Pitch-Scan) ----
python3 ~/ws_lidar/src/ros2-lidar-explorer/python/servo.py \
  --ros-args \
    -p calib_yaml:=/home/robi/ws_lidar/src/ros2-lidar-explorer/config/calibrate.yaml \
    -p pendulum_deg:=15.0 &
bg_pid9=$!

# ---- Scan → PointCloud2 ----
python3 ~/ws_lidar/src/ros2-lidar-explorer/python/scan_to_pointcloud.py &
bg_pid10=$!

# Warten bis die PointCloud verfügbar ist
until ros2 topic echo /servo_lidar/pointcloud --once >/dev/null 2>&1; do
  echo "⏳ Waiting for /servo_lidar/pointcloud..."
  sleep 1
done

# ---- OctoMap Server ----
ros2 run octomap_server octomap_server_node \
  --ros-args \
  --params-file /home/robi/ws_lidar/src/ros2-lidar-explorer/config/octomap.yaml \
  -r /cloud_in:=/servo_lidar/pointcloud &
bg_pid11=$!

cd ~/ws_lidar/src/ros2-lidar-explorer/python

python3 navigate_relay.py &
bg_pid12=$!

sleep 1

python3 relaunch.py "$MAP_NAME" &
bg_pid3=$!

wait
