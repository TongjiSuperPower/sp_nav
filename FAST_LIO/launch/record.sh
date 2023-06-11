source /opt/ros/noetic/setup.bash
source /home/rm/ws_livox2/devel/setup.bash
source /home/rm/catkin_ws/devel/setup.bash
gnome-terminal -- bash -c "bash /home/rm/catkin_ws/src/FAST_LIO/launch/start.sh "& sleep 5s
gnome-terminal -- bash -c "bash /home/rm/catkin_ws/src/FAST_LIO/launch/mapping_360.sh "
#gnome-terminal -- bash -c "cd /home/rm/catkin_ws/src/FAST_LIO/PCD & rosbag record -O /home/rm/catkin_ws/src/FAST_LIO/PCD/bag /livox/lidar /livox/imu"
gnome-terminal -- bash -c "python3 ~/catkin_ws/src/FAST_LIO/scripts/record_odometry.py"
