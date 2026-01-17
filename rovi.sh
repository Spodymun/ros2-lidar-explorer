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
    MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
fi

# 3D-Mapping Option
read -p "3D-Mapping aktivieren? (j/n) [j]: " USE_3D_MAPPING
USE_3D_MAPPING="${USE_3D_MAPPING:-j}"

echo "Räume alte Daten auf..."
rm -f ~/.ros/pose_db.db ~/.ros/slam_toolbox*db ~/.ros/seen_frontiers* ~/.ros/*.yaml ~/.ros/*.pgm

# 1) Robot State Publisher (URDF)
if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  URDF_FILE="robot.urdf_3d.xacro"
else
  URDF_FILE="robot.urdf.xacro"
fi

gnome-terminal -- bash -c "\
  echo '=== Robot State Publisher ($([ \"$USE_3D_MAPPING\" = \"j\" ] && echo \"3D\" || echo \"2D\")) ==='; \
  ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:=\"\$(xacro ~/ws_lidar/src/ros2-lidar-explorer/description/$URDF_FILE)\" \
    -p use_sim_time:=false; \
  exec bash" &

sleep 1

# 2) Motor Publisher (Single Source of Truth - Odometry + Joint States)
gnome-terminal -- bash -c "\
  echo '=== Motor Publisher (Unified) ==='; \
  cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
  python3 motor_publisher.py $ARDUINO_PORT; \
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
if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  RVIZ_FILE="3d.rviz"
else
  RVIZ_FILE="explore.rviz"
fi

gnome-terminal -- bash -c "\
  echo '=== RViz ==='; \
  rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/$RVIZ_FILE; \
  exec bash" &

sleep 2

# 6) CMD Vel to Motors Controller
gnome-terminal -- bash -c "\
  echo '=== CMD Vel to Motors ==='; \
  cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
  python3 cmd_vel_to_motors.py $ARDUINO_PORT; \
  exec bash" &

sleep 1

# 7) Twist Mux (Command Multiplexer)
gnome-terminal -- bash -c "\
  echo '=== Twist Mux ==='; \
  ros2 launch ros2-lidar-explorer twist_mux_launch.py; \
  exec bash" &

sleep 1

# 8) Navigate Relay (wichtig für Nav2 Integration!)
gnome-terminal -- bash -c "\
  echo '=== Navigate Relay ==='; \
  cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
  python3 navigate_relay.py; \
  exec bash" &

sleep 1

# 9) Nav2 Navigation Stack
if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  NAV2_PARAMS="nav2_params_3d_pi.yaml"
else
  NAV2_PARAMS="nav2_params.yaml"
fi

gnome-terminal -- bash -c "\
  echo '=== Nav2 Navigation ==='; \
  ros2 launch nav2_bringup navigation_launch.py \
    autostart:=true \
    use_lifecycle_mgr:=true \
    params_file:=$HOME/ws_lidar/src/ros2-lidar-explorer/config/$NAV2_PARAMS; \
  exec bash" &

sleep 2

# 10) Teleop Keyboard
gnome-terminal -- bash -c "\
  echo '=== Keyboard Teleop ==='; \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; \
  exec bash" &

sleep 1

# ============ 3D MAPPING COMPONENTS (Falls aktiviert) ============

if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  echo "🚀 3D-Mapping aktiviert!"
  
  # 11) Servo Control
  gnome-terminal -- bash -c "\
    echo '=== Servo Control ==='; \
    cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
    python3 servo.py \
      --ros-args \
        -p calib_yaml:=$HOME/ws_lidar/src/ros2-lidar-explorer/config/calibrate.yaml \
        -p pendulum_deg:=15.0; \
    exec bash" &
  
  sleep 2
  
  # 12) Scan to PointCloud Converter
  gnome-terminal -- bash -c "\
    echo '=== Scan to PointCloud ==='; \
    cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
    python3 scan_to_pointcloud.py; \
    exec bash" &
  
  sleep 2
  
  # 13) OctoMap Server (3D Mapping)
  gnome-terminal -- bash -c "\
    echo '=== OctoMap Server ==='; \
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file $HOME/ws_lidar/src/ros2-lidar-explorer/config/octomap.yaml \
      -r /cloud_in:=/servo_lidar/pointcloud; \
    exec bash" &
  
  sleep 2

else
  echo "2D-Mapping aktiv"
fi

echo "✅ Alle Services gestartet!"
echo "Drücke Ctrl+C zum Beenden..."

wait