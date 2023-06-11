sudo ip link set can0 up type can bitrate 1000000
source ~/catkin_ws/devel/setup.bash
rosrun sentry_communicator sentry_communicator

