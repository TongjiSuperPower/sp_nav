C板通过USB2CAN模块连接电脑，开调试

ROS开内核：`roscore`

开CAN口：`sudo ip link set can0 up type can bitrate 1000000`

把sentry_communicator拖进虚拟机，在对应文件夹里输入：`rosrun sentry_communicator sentry_communicator`

开新终端，发布话题：

`rostopic pub -r 60 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"`