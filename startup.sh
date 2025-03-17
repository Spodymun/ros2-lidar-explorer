#!/bin/bash

# Prüfen, ob eine IP-Adresse als Argument übergeben wurde
if [ -z "$1" ]; then
    echo "Fehlende IP-Adresse! Nutzung: ./startup.bash <IP>"
    exit 1
fi

ESP_IP=$1
echo "Verwende IP-Adresse: $ESP_IP"

# Cleanup-Funktion zum Beenden aller gestarteten Prozesse
cleanup() {
    echo "Beende alle Hintergrundprozesse..."
    kill $bg_pid1
    kill $bg_pid2
    kill $bg_pid3
    kill $bg_pid4
    kill $bg_pid5
    kill $bg_pid6
}

# Sicherstellen, dass die Cleanup-Funktion beim Beenden des Skripts ausgeführt wird
trap cleanup EXIT

# Starten der ROS2-Befehle mit übergebener IP

# 1. Starten des Robot Launch Scripts mit IP
ros2 launch ros2-lidar-explorer launch_robot.launch.py esp_ip:="$ESP_IP" &
bg_pid1=$!
echo "Prozess 1 (launch_robot.launch.py) im Hintergrund gestartet, PID: $bg_pid1"

# 2. Starten des RPLidar Scripts
ros2 launch ros2-lidar-explorer rplidar.launch.py &
bg_pid2=$!
echo "Prozess 2 (rplidar.launch.py) im Hintergrund gestartet, PID: $bg_pid2"

# 3. Starten von SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py &
bg_pid3=$!
echo "Prozess 3 (online_async_launch.py) im Hintergrund gestartet, PID: $bg_pid3"

# 4. Starten von teleop_twist_keyboard in einem neuen Terminal
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash" &
bg_pid4=$!
echo "Prozess 4 (teleop_twist_keyboard) in einem neuen Terminal gestartet, PID: $bg_pid4"

# 5. Starten des ESP Control Scripts mit der übergebenen IP
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 esp_http_control_test.py "$ESP_IP" &
bg_pid5=$!
echo "Prozess 5 (esp_http_control.py) mit IP $ESP_IP im Hintergrund gestartet, PID: $bg_pid5"

# 6. Starten von RVIZ2 in einem neuen Terminal
gnome-terminal -- bash -c "rviz2; exec bash" &
bg_pid6=$!
echo "Prozess 4 (rviz2) in einem neuen Terminal gestartet, PID: $bg_pid6"

# Warten, damit das Skript nicht sofort beendet wird
wait