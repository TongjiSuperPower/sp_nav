source ~/ws_livox2/devel/setup.bash
{
gnome-terminal -x bash -c "roscore;exec bash"
}&
sleep 3s
{
gnome-terminal -x bash -c "roslaunch livox_ros_driver2 msg_MID360.launch;exec bash"
}





