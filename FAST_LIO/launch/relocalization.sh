source ~/catkin_ws/devel/setup.bash
{
gnome-terminal -x bash -c "roslaunch relocalization_mid_360.launch;exec bash"
}&
sleep 3s
{
gnome-terminal -x bash -c "roslaunch relocalization_octomap.launch;exec bash"
}&
{
gnome-terminal -x bash -c "roslaunch map_save.launch;exec bash"
}&


