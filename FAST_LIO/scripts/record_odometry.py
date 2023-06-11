import rospy
from nav_msgs.msg import Odometry

def callback(data):
    # 在回调函数中处理接收到的消息

    # 将消息保存到txt文件
    with open('~/catkin_ws/src/FAST-LIO/PCD/odometry.txt', 'a') as file:
        file.write(str(data) + '\n')

def subscribe_odometry():
    # 初始化ROS节点
    rospy.init_node('odometry_subscriber', anonymous=True)

    # 创建一个订阅者，订阅名为'/odom'的Odometry消息
    rospy.Subscriber('/Odometry', Odometry, callback)

    # 设置循环的频率为0.5Hz
    rate = rospy.Rate(0.5)

    # 循环等待消息到达
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    try:
        subscribe_odometry()
    except rospy.ROSInterruptException:
        pass

