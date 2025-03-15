#!/bin/bash

# Funktion, um alle Hintergrundprozesse zu beenden
cleanup() {
    echo "Beende alle Hintergrundprozesse..."
    kill $bg_pid1
    kill $bg_pid2
    kill $bg_pid3
    kill $bg_pid4
    kill $bg_pid5
    kill $bg_pid6
}

# Sicherstellen, dass beim Beenden des Skripts die Cleanup-Funktion ausgeführt wird
trap cleanup EXIT

# Starten der ROS2-Befehle im Hintergrund

# Befehl 1: ros2 launch ros2-lidar-explorer launch_robot.launch.py
ros2 launch ros2-lidar-explorer launch_robot.launch.py &
bg_pid1=$!
echo "Prozess 1 (launch_robot.launch.py) im Hintergrund gestartet, PID: $bg_pid1"

# Befehl 2: ros2 launch ros2-lidar-explorer rplidar.launch.py
ros2 launch ros2-lidar-explorer rplidar.launch.py &
bg_pid2=$!
echo "Prozess 2 (rplidar.launch.py) im Hintergrund gestartet, PID: $bg_pid2"

# Befehl 3: ros2 launch slam_toolbox online_async_launch.py
ros2 launch slam_toolbox online_async_launch.py &
bg_pid3=$!
echo "Prozess 3 (online_async_launch.py) im Hintergrund gestartet, PID: $bg_pid3"

## Befehl 4: Starte teleop_twist_keyboard in einem neuen Terminal
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash" &
bg_pid4=$!
echo "Prozess 4 (teleop_twist_keyboard) in einem neuen Terminal gestartet, PID: $bg_pid4"

# Befehl 5: cd ~/ws_lidar/src/ros2-lidar-explorer/python und python3 esp_http_control.py
cd ~/ws_lidar/src/ros2-lidar-explorer/python && python3 esp_http_control.py &
bg_pid5=$!
echo "Prozess 5 (esp_http_control.py) im Hintergrund gestartet, PID: $bg_pid5"

# Befehl 6: rviz2
rviz2 &
bg_pid6=$!
echo "Prozess 6 (rviz2) im Hintergrund gestartet, PID: $bg_pid6"

# Warten, damit das Skript nicht sofort beendet wird
wait
