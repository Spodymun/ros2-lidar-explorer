#!/usr/bin/env bash

# Arduino-basiertes SLAM + Exploration
# Nutzt Arduino für 4-Motor Odometrie (PWM + Encoder)
# Minimierte Terminal-Ansicht

# Arduino Port überprüfen
ARDUINO_PORT="${1:-/dev/ttyACM2}"
if [ ! -e "$ARDUINO_PORT" ]; then
    ARDUINO_PORT="/dev/ttyUSB0"
fi

if [ ! -e "$ARDUINO_PORT" ]; then
    echo "❌ Arduino nicht gefunden!"
    ls /dev/tty{ACM,USB}* 2>/dev/null
    exit 1
fi

echo "✅ Arduino gefunden: $ARDUINO_PORT"

# ROS env
source /opt/ros/jazzy/setup.bash
source ~/ws_lidar/install/setup.bash

# Servo VENV (falls vorhanden)
if [ -f ~/ws_lidar/src/STServo_Python/venv-servo/bin/activate ]; then
    source ~/ws_lidar/src/STServo_Python/venv-servo/bin/activate
fi

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

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro ~/ws_lidar/src/ros2-lidar-explorer/description/robot.urdf.xacro)" \
  -p use_sim_time:=false > /dev/null 2>&1 &
echo "[1] Robot State Publisher"

sleep 1

# 2) Motor Publisher
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 motor_publisher.py $ARDUINO_PORT > /dev/null 2>&1 &
echo "[2] Motor Publisher"

sleep 1

# 3) LIDAR Scan
ros2 launch sllidar_ros2 sllidar_a2m8_launch.py > /dev/null 2>&1 &
echo "[3] LIDAR Scan"

sleep 2

# 4) SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false > /dev/null 2>&1 &
echo "[4] SLAM Toolbox"

sleep 3

# 5) RViz (INTERAKTIV - braucht Terminal!)
if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  RVIZ_FILE="3d.rviz"
else
  RVIZ_FILE="explore.rviz"
fi

gnome-terminal -- bash -c "\
  echo '=== RViz ==='; \
  rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/$RVIZ_FILE; \
  exec bash" &
echo "[5] RViz (Terminal)"

sleep 2

# 6) CMD Vel to Motors Controller
python3 cmd_vel_to_motors.py $ARDUINO_PORT > /dev/null 2>&1 &
echo "[6] CMD Vel to Motors"

sleep 1

# 7) Twist Mux
ros2 launch ros2-lidar-explorer twist_mux_launch.py > /dev/null 2>&1 &
echo "[7] Twist Mux"

sleep 1

# 8) Navigate Relay
python3 navigate_relay.py > /dev/null 2>&1 &
echo "[8] Navigate Relay"

sleep 1

# 9) Servo Control (immer - bei 2D mit 0° pendulum, bei 3D mit 15° pendulum)
if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  PENDULUM_DEG="15.0"
  DELTA_DEG="0.5"
else
  PENDULUM_DEG="0.0"  # Bei 2D: Servo statisch, aber sichtbar
  DELTA_DEG="0.0"     # Keine Bewegung!
fi

python3 servo.py \
  --ros-args \
    -p calib_yaml:=$HOME/ws_lidar/src/ros2-lidar-explorer/config/calibrate.yaml \
    -p pendulum_deg:=$PENDULUM_DEG \
    -p delta_deg:=$DELTA_DEG > /dev/null 2>&1 &
echo "[9] Servo Control (pendulum: $PENDULUM_DEG°, delta: $DELTA_DEG°)"

sleep 1

# 10) Nav2 Navigation Stack
if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  NAV2_PARAMS="nav2_params_3d_pi.yaml"
else
  NAV2_PARAMS="nav2_params.yaml"
fi

ros2 launch nav2_bringup navigation_launch.py \
  autostart:=true \
  use_lifecycle_mgr:=true \
  params_file:=$HOME/ws_lidar/src/ros2-lidar-explorer/config/$NAV2_PARAMS > /dev/null 2>&1 &
echo "[10] Nav2 Navigation"

sleep 2

# 11) Teleop Keyboard (INTERAKTIV - braucht Terminal!)
gnome-terminal -- bash -c "\
  echo '=== Keyboard Teleop ==='; \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; \
  exec bash" &
echo "[11] Keyboard Teleop (Terminal)"

sleep 1

# ============ 3D MAPPING COMPONENTS (Falls aktiviert) ============

if [ "$USE_3D_MAPPING" = "j" ] || [ "$USE_3D_MAPPING" = "J" ]; then
  echo ""
  echo "🚀 3D-Mapping aktiviert!"
  
  # 12) Scan to PointCloud Converter
  python3 scan_to_pointcloud.py > /dev/null 2>&1 &
  echo "[12] Scan to PointCloud"
  
  sleep 2
  
  # 13) OctoMap Server (3D Mapping)
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    --params-file $HOME/ws_lidar/src/ros2-lidar-explorer/config/octomap.yaml \
    -r /cloud_in:=/servo_lidar/pointcloud > /dev/null 2>&1 &
  echo "[13] OctoMap Server"

else
  echo "2D-Mapping aktiv"
fi

echo ""
echo "✅ Alle Services gestartet!"
echo "� Services laufen im Hintergrund (nur RViz + Teleop in Terminals)"
echo "Drücke Ctrl+C zum Beenden..."

wait