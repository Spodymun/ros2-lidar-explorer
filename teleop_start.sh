#!/usr/bin/env bash

# Arduino-basiertes SLAM + Exploration
# Nutzt Arduino für 4-Motor Odometrie (PWM + Encoder)
# Minimierte Terminal-Ansicht

# ROS env
source /opt/ros/jazzy/setup.bash
source ~/ws_lidar/install/setup.bash

# 2) Motor Publisher
cd ~/ws_lidar/src/ros2-lidar-explorer/python
python3 motor_publisher.py ACM1 > /dev/null 2>&1 &
echo "[2] Motor Publisher"

sleep 1

 # 6) CMD Vel to Motors Controller
python3 cmd_vel_to_motors.py ACM1 > /dev/null 2>&1 &
echo "[6] CMD Vel to Motors"

sleep 1

# 7) Twist Mux
ros2 launch ros2-lidar-explorer twist_mux_launch.py > /dev/null 2>&1 &
echo "[7] Twist Mux"

sleep 1


# 11) Teleop Keyboard (INTERAKTIV - braucht Terminal!)
gnome-terminal -- bash -c "\
  echo '=== Keyboard Teleop ==='; \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop; \
  exec bash" &
echo "[11] Keyboard Teleop (Terminal)"
