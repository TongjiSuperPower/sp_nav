# 该记录已经废弃，修改记录保存在私有仓库https://github.com/ailiwei17/FAST-LIO
| 任务 | 原因 | 进度 |
| --- | --- | --- |
| 调试导航算法参数 | 主要调试costmap的参数，保证避障的同时不使机器人卡死 |  |

## Commit: 增加了保存位姿和画图程序
## Commit: 增加了急停状态
* 修复了Invalid robot state value
## Commit: 增加了录屏
## Commit: 对抗赛测试
* 调整了最低速度
## Commit: 移动测试2
* 增加了动态/静态的膨胀系数
* 增加了FAST状态
* 减小了地图体素化分辨率
## Commit: 移动道具测试
* 增加了避障频率
## Commit: 场地道具测试1
* 删除了联盟赛才会用到的状态->测试通过
* 对速度的限制不再分x,y轴，而是限制后再根据航向角转换到x,y轴->测试通过
* 提高控制频率，减小距离->测试通过
* 较小的膨胀系数:0.4 较大的膨胀系数:0.5
* TODO：优化减速点检测

## Commit: 优化算法
* 删除了联盟赛才会用到的状态
* 对速度的限制不再分x,y轴，而是限制后再根据航向角转换到x,y轴
* TODO：提高控制频率，减小距离
* TODO：优化减速点检测

## Commit:  调整参数
* OUTPOST减速设置
* 膨胀系数设置

## Commit:  使用robot_msg作为消息包
## Commit:  联盟赛
## Commit:  增加策略
## Commit:  稳定测试
## Commit:  低分辨率清除障碍/修复路径锯齿
## Commit:  增加代码复用性/增加延时
## Commit:  增加机器人状态维护/策略切换
## Commit:  3v3 test
## Commit:  尝试修改octomap
## Commit:  在线切换策略/开机自启动/实时建图测试
## Commit:  开机自启动
## Commit:  切换不同的策略
* 用于切换两种导航模式(待测试)，参数在param/navigation_model
## Commit:  test_success
* 一个稳定的版本，初步实现了遇到障碍物减速以及避让的功能
## Commit:  两点巡航改进
* 在遇到障碍物时减速
## Commit:  发布空航路点
* 到达终点后，航路点依然存在
## Commit:  实际测试2
* 更新了base_link_pose发布不正确的情况
## Commit:  实际测试
* 修改了move_base 参数和 octomap参数
* 重新设置了定位成功的判断，不再使用TF直接判断
## Commit:  增加了对底盘与云台分离的支持
* 把点云转换到世界坐标系下,用于输入local_planner
* 当云台与底盘固定时，body->base_link由静态变换给出
* 当云台与底盘不固定时，body->base_link由use2can给出
* 修改了跟踪参数
## Commit: cmu_local_planner
* 通过局部路径规划插件，更新了对cmu_local_planner的支持，通过订阅A*，取5米远的点作为航路点，并且新生成的航路点距离旧航路点1米时才更新
* cmu_local_planner发送的话题为/cmd_vel_
## Commit: 参数修改
* 修改了一些导航参数
## Commit: test_local_planner
* rosparam没有加载成功，注意参数名
## Commit: new_local_planner
* 把vel_control集成为move_base local_planner，但具体效果还没有上车测试
## Commit: 修改导航成功条件
* 导航完成条件，已经修正
* 这个commit通过了vel_control的测试
## Commit: test_vel_control_06
* 航向角定义错误，已经修正
## Commit: test_vel_control_05
* 把x,y方向PID合成为距离PID
## Commit: test_vel_control_04
*  cmd_vel.linear.x = std::cos(yaw) * x_output - std::sin(yaw) * y_output;  
   cmd_vel.linear.y = std::sin(yaw) * x_output + std::cos(yaw) * y_output;  
   yaw必须是弧度  
## Commit: test_vel_control_03
* 目前的解决方案是：把旧的全局路径存起来
* 倒走时有问题是因为跟踪的局部路径与位姿有关
## Commit: test_vel_control_02
* 修改了距离判定中的PID，路径最后一点不一定距离机器人最远
* 航向角分解，倒走时有问题
* 需要写一个条件，提前向Move_Base发送到达指令
* 卡顿的原因是全局路径规划更新频率较低，进入路径回调函数的频率也比较低
## Commit: test_vel_control
* PID输出会周期性出现0值，通过调整控制频率到5Hz解决
* 在到达终点后，偶发向外输出速度信号，在终点附近振荡。
1. 在到达目标点后，停止机器人，并设置目标点为机器人当前位置。
2. 速度控制是机器人坐标系，而代码中使用的是世界坐标系
* 绘图脚本scripts/vel_plotter.py检测到x,y,z数据长度不同,通过pad操作把数据统一到最大长度，并把缺少数据补充为nan解决












