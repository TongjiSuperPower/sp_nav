import matplotlib.pyplot as plt

x_values = []
y_values = []

# 读取txt文件
with open('~/catkin_ws/src/FAST-LIO/PCD/odometry.txt', 'r') as file:
    for line in file:
        if 'pose' in line:
            # 提取x和y的数值
            x_index = line.index('x: ') + 3
            y_index = line.index('y: ') + 3
            x = float(line[x_index : x_index + 8])
            y = float(line[y_index : y_index + 8])

            # 添加到数据列表中
            x_values.append(x)
            y_values.append(y)

# 绘制图片
plt.plot(x_values, y_values)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Odometry Trajectory')
plt.show()

