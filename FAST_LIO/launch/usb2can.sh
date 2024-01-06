sudo ip link set can0 up type can bitrate 1000000
source ~/sp_nav_ws/devel/setup.bash
rosrun sentry_communicator sentry_communicator

