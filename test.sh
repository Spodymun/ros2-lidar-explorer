#!/usr/bin/env bash

# 1) Karte wählen
read -p "Karten-Verzeichnis unter maps/: " MAP
MAP_YAML="$HOME/ws_lidar/src/ros2-lidar-explorer/maps/$MAP/map.yaml"
if [ ! -f "$MAP_YAML" ]; then
  echo "❌ Fehler: $MAP_YAML existiert nicht!"
  exit 1
fi

# 2) ESP-IP abfragen
read -p "ESP-IP-Adresse (z.B. 192.168.0.10): " ESP_IP

# 4) Robot Launch
gnome-terminal -- bash -c "\
  echo '=== Robot Launch ==='; \
  ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:=$ESP_IP; \
  exec bash"

# 3) ESP HTTP Control starten (mit IP)
gnome-terminal -- bash -c "\
  echo '=== ESP HTTP Control (IP: $ESP_IP) ==='; \
  cd ~/ws_lidar/src/ros2-lidar-explorer/python; \
  python3 esp_http_control.py $ESP_IP; \
  exec bash"

# 5) SLLIDAR A2M8 Driver
gnome-terminal -- bash -c "\
  echo '=== SLLIDAR A2M8 Driver ==='; \
  ros2 launch sllidar_ros2 sllidar_a2m8_launch.py; \
  exec bash"

gnome-terminal -- bash -c "rviz2 -d ~/ws_lidar/src/ros2-lidar-explorer/rviz/localization.rviz; exec bash" &

# 6) kurze Pause, damit der Lidar-Stack ready ist
sleep 5

# 7) Nav2-Bringup (Map-Server + AMCL + Planner…)
gnome-terminal -- bash -c "\
  echo '=== Nav2 Navigation Stack ==='; \
  ros2 launch nav2_bringup localization_launch.py \
    map:=$MAP_YAML \
    params_file:=$HOME/ws_lidar/src/ros2-lidar-explorer/config/amcl_params.yaml \
    use_sim_time:=False; \
  exec bash"

echo "✅ Alle Komponenten werden in separaten Terminals gestartet."
