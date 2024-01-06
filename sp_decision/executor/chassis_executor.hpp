/**
 * @file chassis_executor.hpp
 * @author liwei
 * @brief  发送给sp_planning的目标点以及机器人状态，用于代替aerial_navigation
 * @version 0.1
 * @date 2023-06-13
 * DANGER: 该段代码还没有在虚拟/实际的裁判系统上测试
 */
#ifndef CHASSIS_EXECUTOR_H
#define CHASSIS_EXECUTOR_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_msg/RobotStateMsg.h>
#include <ros/ros.h>


//给MoveBaseAction定义一个别名，方便创建对象
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class ChassisExecutor {
 public:
  enum RobotState {
    MOVE,
    CRUISR,
    IDLE,
    FAST,
    STOP,
  };
  ChassisExecutor(){

    nh_.param<double>("/max_vel_theta", max_vel_theta_, 3.14);
    set_goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    robot_state_pub_ =
        nh_.advertise<robot_msg::RobotStateMsg>("/robot_state", 1);
    // TODO：
    // 在这里出现速度控制器十分不合理，但是sp_planning是以局部规划器的形式写在move_base框架中，当没有目标点时，里面的速度规划器不会工作
    sentry_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("sentry/cmd_vel", 1);
  }
  typedef std::shared_ptr<ChassisExecutor> Ptr;

  void robotStatePub(RobotState robot_state) {
    robot_msg::RobotStateMsg robot_state_msg;
    robot_state_msg.robot_state = static_cast<int8_t>(robot_state);
    robot_state_pub_.publish(robot_state_msg);
  }
  void Move(double pos_x, double pos_y) {
    robotStatePub(RobotState::MOVE);
    SendDataToPlan(pos_x, pos_y);
  }
  void FastMove(double pos_x, double pos_y) {
    robotStatePub(RobotState::FAST);
    SendDataToPlan(pos_x, pos_y);
  }
  void Cruisr(double pos_x, double pos_y) {
    robotStatePub(RobotState::CRUISR);
    SendDataToPlan(pos_x, pos_y);
  }
  void VelIdle() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = max_vel_theta_;
    sentry_vel_pub_.publish(cmd_vel);
  }
  void VelStop() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    sentry_vel_pub_.publish(cmd_vel);
  }
  void Stop() {
    robotStatePub(RobotState::STOP);
    VelStop();
  }
  void Idle() {
    robotStatePub(RobotState::IDLE);
    VelIdle();
  }
  void SendDataToPlan(double pos_x, double pos_y) {

    // if (target_pose_.pose.position.x == pos_x && target_pose_.pose.position.y == pos_y) return;
   
    target_pose_.header.frame_id = "map";
    target_pose_.header.stamp = ros::Time::now();
    target_pose_.pose.position.x = pos_x;
    target_pose_.pose.position.y = pos_y;
    target_pose_.pose.orientation.x = 0.0;
    target_pose_.pose.orientation.y = 0.0;
    target_pose_.pose.orientation.z = 0.0;
    target_pose_.pose.orientation.w = 1.0;
    goal_.target_pose = target_pose_;
    set_goal_pub_.publish(goal_);
    sentry_vel_pub_.publish(vel_msg_pub_);

  }
  geometry_msgs::Twist vel_msg_pub_;
 private:
  ros::NodeHandle nh_;
  ros::Publisher set_goal_pub_;
  ros::Publisher robot_state_pub_;
  ros::Publisher sentry_vel_pub_;
  geometry_msgs::PoseStamped target_pose_;
  move_base_msgs::MoveBaseGoal goal_;
  double max_vel_theta_;

};

 
#endif