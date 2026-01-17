#!/usr/bin/env bash

# Arduino-basiertes SLAM + Exploration
# Nutzt Arduino für 4-Motor Odometrie (PWM + Encoder)

# Arduino Port überprüfen
ARDUINO_PORT="${1:-/dev/ttyACM0}"
if [ ! -e "$ARDUINO_PORT" ]; then
    ARDUINO_PORT="/dev/ttyUSB0"
fi

if [ ! -e "$ARDUINO_PORT" ]; then
    echo "❌ Arduino nicht gefunden!"
    ls /dev/tty{ACM,USB}* 2>/dev/null
    exit 1
fi

echo "✅ Arduino gefunden: $ARDUINO_PORT"

# Karten-Name
read -p "Karten-Name: " MAP_NAME
if [ -z "$MAP_NAME" ]; then
    MAP_NAME="arduino_map_$(date +%Y%m%d_%H%M%S)"
fi

echo "Räume alte Daten auf..."
rm -f ~/.ros/pose_db.db ~/.ros/slam_toolbox*db ~/.ros/seen_frontiers* ~/.ros/*.yaml ~/.ros/*.pgm

# 1) Robot State Publisher (URDF)
gnome-terminal -- bash -c "\
  echo '=== Robot State Publisher ==='; \
  ros2 launch ros2-lidar-explorer rsp.launch.py; \
  exec bash" &

sleep 1

# 2) Arduino Odometry Node (4 Motor PWM + Encoder)
gnome-terminal -- bash -c "\
  echo '=== Arduino Odometry (Motor PWM basiert) ==='; \
  cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
  python3 arduino_odom_publisher.py $ARDUINO_PORT; \
  exec bash" &

sleep 1

# 2b) Wheel Joint Publisher (animiert die Räder in RVIZ)
gnome-terminal -- bash -c "\
  echo '=== Wheel Joint Publisher ==='; \
  cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
  python3 wheel_joint_publisher.py; \
  exec bash" &

sleep 1

# 3) LIDAR Scan
gnome-terminal -- bash -c "\
  echo '=== SLLIDAR A2M8 ==='; \
  ros2 launch sllidar_ros2 sllidar_a2m8_launch.py; \
  exec bash" &

sleep 2

# 4) SLAM Toolbox (Online Mapping)
gnome-terminal -- bash -c "\
  echo '=== SLAM Toolbox (Online Mapping) ==='; \
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false; \
  exec bash" &

sleep 3

# 5) RViz Visualisierung
gnome-terminal -- bash -c "\
  echo '=== RViz ==='; \
  rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/explore.rviz; \
  exec bash" &

sleep 2

# 6) Twist Mux (Command Multiplexer)
gnome-terminal -- bash -c "\
  echo '=== Twist Mux ==='; \
  ros2 launch ros2-lidar-explorer twist_mux_launch.py; \
  exec bash" &

sleep 1

# 7) Nav2 Navigation Stack
gnome-terminal -- bash -c "\
  echo '=== Nav2 Navigation ==='; \
  ros2 launch nav2_bringup navigation_launch.py \
    autostart:=true \
    use_lifecycle_mgr:=true \
    params_file:=$HOME/ws_lidar/src/ros2-lidar-explorer/config/nav2_params_3d_pi.yaml; \
  exec bash" &

sleep 2

# 8) Teleop Keyboard
gnome-terminal -- bash -c "\
  echo '=== Keyboard Teleop ==='; \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; \
  exec bash" &

echo "✅ Alle Systeme gestartet!"