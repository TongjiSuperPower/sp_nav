/**
 * @file navigation_model.h
 * @author liwei
 * @brief  状态切换
 * @version 0.2
 * @date 2023-04-06
 * DANGER: 该段代码的机器人状态为联盟赛时期维护，现已废弃，留存目的是给chen heming以及liwei学习使用，新状态请参考sp_local_planner以及sp_decison
 */
#ifndef NAVIGATION_MODEL_H
#define NAVIGATION_MODEL_H
 
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <vector>
#include <deque>
#include <std_msgs/Bool.h>
#include <robot_msg/RobotStateMsg.h>

//给MoveBaseAction定义一个别名，方便创建对象
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
 
class GoalSending {
 private:
  // ros
  ros::NodeHandle nh_;
  ros::Subscriber localization_sub_;
  ros::Publisher robot_state_pub_;
  
  geometry_msgs::PoseStamped target_pose_;
  move_base_msgs::MoveBaseGoal goal_;
  // 创建MoveBaseAction对象
  Client ac_;
  
  // 定义机器人状态
  bool localization_success_;
  enum RobotState {
    OUTPOST,
    CRUISR,
    IDLE,
    DEFENCE,
    OUTPOST_NEXT
  };
  RobotState state_;
  ros::Timer timer_robot_state_;

  ros::Timer timer_state_check_;

  // 原地小陀螺参数
  ros::Publisher vel_pub_;
  // 在切换原地小陀螺状态前，必须更新这三个参数
  ros::Time idle_start_;
  double first_duration_;
  double second_duration_;
  // 如果是最后一个点，duration_无效
  bool end_goal_; 
  RobotState next_state_;
  double duration_;
  double max_vel_theta_;

  // 回防参数
  std::deque<std::vector<double>> defence_goal_;
  // 前哨站参数
  std::deque<std::vector<double>> outpost_goal_;
  // 再次到前哨站
  std::deque<std::vector<double>> outpost_next_goal_;

  // 两点巡航参数
  bool cruise_initialized_;
  int count_ = 0;

  std::vector<double> first_goal_;
  std::vector<double> second_goal_;

  /**
   * @brief 持续发布机器人状态
   */
  void robotStatePub(const ros::TimerEvent& event);

  /**
   * @brief 监听定位状态
   * @param msg 定位消息
   */
  void localizationCallback(const std_msgs::Bool::ConstPtr& msg);
  /**
   * @brief 设置机器人状态
   * @param state 机器人状态
   */
  void setState(RobotState state);

  /**
   * @brief 发布依次巡迹目标点
   * @param goal_list 目标点列表
   * @param cur_state 当前机器人状态
   * @param next_state 下一个机器人状态
  */
  void popSequential(std::deque<std::vector<double>> &goal_list, RobotState cur_state,RobotState next_state);

  /**
   * @brief 原地小陀螺执行函数
   */
  void idle();

  /**
   * @brief 判断机器人状态
   */
  void stateCheck(const ros::TimerEvent& event);
 
  /**
   * @brief 两点巡航执行函数
   */
  void twoPointCruise();
  /**
   * @brief 读取目标点
   */
  void pointRead();
  //判断是否接收到目标点
  void activeCb();
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const move_base_msgs::MoveBaseResultConstPtr& result);
  
 public:
  GoalSending();
};
 
#endif  // NAVIGATION_MODEL_H