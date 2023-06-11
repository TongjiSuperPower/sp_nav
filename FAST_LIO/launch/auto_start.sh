#!/bin/bash
sleep 5
source /opt/ros/noetic/setup.bash
source /home/rm/ws_livox2/devel/setup.bash
source /home/rm/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/rm/ws_livox2/src:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rm/catkin_ws/src:$ROS_PACKAGE_PATH
gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch "& sleep 5s
gnome-terminal -- bash -c "roslaunch fast_lio relocalization_nav.launch  "& sleep 1s
#gnome-terminal -- bash -c "roslaunch fast_lio mapping_mid_360_nav.launch  "& sleep 1s
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
gnome-terminal -- bash -c "rosrun sentry_communicator sentry_communicator "& sleep 1s
bash /home/rm/RM23_CV_TJU-main/autostart.sh
