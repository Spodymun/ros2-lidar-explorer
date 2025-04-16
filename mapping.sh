#!/bin/bash

if [ -z "$1" ]; then
    echo "Missing IP address! Usage: ./startup.bash <IP>"
    exit 1
fi

ESP_IP=$1
echo "Using IP address: $ESP_IP"

read -p "What's the map called? " MAP_NAME
echo "Using map name: $MAP_NAME"

echo "Cleaning previous SLAM and navigation data..."
rm -f ~/.ros/pose_db.db ~/.ros/slam_toolbox*db ~/.ros/seen_frontiers* ~/.ros/*.yaml ~/.ros/*.pgm

cleanup() {
    echo "Shutting down background processes..."
    kill $bg_pid1 $bg_pid2 $bg_pid3 $bg_pid4 $bg_pid5 $bg_pid6 \
         $bg_pid7 $bg_pid8 $bg_pid9 $bg_pid10 $bg_pid11 #$bg_pid12
}
trap cleanup EXIT

ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!
sleep 3

ros2 launch sllidar_ros2 sllidar_a2m8_launch.py &
bg_pid2=$!

# until ros2 topic echo /scan_raw --once; do
#   echo "Waiting for /scan_raw..."
#   sleep 1
# done
# echo "✅ /scan_raw detected"

cd ~/ws_lidar/src/ros2-lidar-explorer/python
# python3 scan_timestamp_relay.py &
# bg_pid10=$!

until ros2 topic echo /scan --once; do
  echo "Waiting for /scan..."
  sleep 1
done
echo "✅ /scan detected"

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
bg_pid3=$!
sleep 2

ros2 launch nav2_bringup navigation_launch.py \
  autostart:=true \
  use_lifecycle_mgr:=true \
  params_file:=/home/robi/ws_lidar/src/ros2-lidar-explorer/config/nav2_params.yaml &
bg_pid7=$!
sleep 3

gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/explore.rviz; exec bash" &
bg_pid6=$!

ros2 launch ros2-lidar-explorer twist_mux_launch.py &
bg_pid8=$!

gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; exec bash" &
bg_pid4=$!

python3 esp_http_control.py "$ESP_IP" &
bg_pid5=$!

python3 navigate_relay.py &
bg_pid11=$!

#ros2 launch ros2-lidar-explorer d415.launch.py &
#bg_pid12=$!

sleep 1

python3 relaunch.py "$MAP_NAME" &
bg_pid9=$!

wait
