import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tf2_ros
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import TransformStamped

# 创建一个空的列表来存储航向角度数据
yaw_data = []


def tf_callback(data):
    try:
        trans = tf_buffer.lookup_transform('world', 'base_link', rospy.Time())
        yaw = euler_from_quaternion([
            trans.transform.rotation.x, trans.transform.rotation.y,
            trans.transform.rotation.z, trans.transform.rotation.w
        ])[2]
        # 将航向角度数据添加到列表中
        yaw_data.append(math.degrees(yaw))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass


def update_plot(frame):
    plt.cla()

    yaw_len = len(yaw_data)

    # 将yaw数据pad到相同长度
    max_len = yaw_len
    yaw_pad = np.pad(yaw_data, (0, max_len - yaw_len), 'constant', constant_values=np.nan)

    ax = plt.subplot(111)
    ax.plot(yaw_pad, label='yaw')

    ax.legend()
    ax.set_xlabel('Times')
    ax.set_ylabel('Yaw')

    # 设置画多少值
    ax.set_xlim(max(0, len(yaw_data) - 5000), len(yaw_data))

    plt.draw()


if __name__ == "__main__":
    rospy.init_node('yaw_plotter')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/tf', TransformStamped, tf_callback)

    # 动态画图
    ani = animation.FuncAnimation(plt.gcf(), update_plot, interval=100)

    plt.show()
    rospy.spin()
