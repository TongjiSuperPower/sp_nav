import rospy
import numpy as np
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 初始化数组
x_vels = []
y_vels = []
ang_vels = []


# 回调函数
def callback(data):
    # x,y线速度
    x_vel = data.linear.x
    y_vel = data.linear.y
    ang_vel = data.angular.z

    # 添加到数组中
    x_vels.append(x_vel)
    y_vels.append(y_vel)
    ang_vels.append(ang_vel)


def update_plot(frame):
    plt.cla()

    x_len = len(x_vels)
    y_len = len(y_vels)
    ang_len = len(ang_vels)

    # 将所有数据pad到相同长度
    max_len = max(x_len, y_len, ang_len)
    x_pad = np.pad(x_vels, (0, max_len - x_len), 'constant', constant_values=np.nan)
    y_pad = np.pad(y_vels, (0, max_len - y_len), 'constant', constant_values=np.nan)
    ang_pad = np.pad(ang_vels, (0, max_len - ang_len), 'constant', constant_values=np.nan)

    ax = plt.subplot(111)
    ax.plot(x_pad, label='x_vel')
    ax.plot(y_pad, label='y_vel')
    ax.plot(ang_pad, label='ang_vel')

    ax.legend()
    ax.set_xlabel('Times')
    ax.set_ylabel('Velocity')

    # 设置画多少值
    ax.set_xlim(max(0, len(x_vels) - 5000), len(x_vels))

    plt.draw()


if __name__ == "__main__":
    rospy.init_node('vel_plotter')
    rospy.Subscriber('/cmd_vel', Twist, callback)

    # 动态画图
    ani = animation.FuncAnimation(plt.gcf(), update_plot, interval=100)

    plt.show()

    rospy.spin()
