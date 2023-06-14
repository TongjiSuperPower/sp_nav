/**
 * @file blackboard.hpp
 * @author liwei
 * @brief  数据集合
 * @version 0.1
 * @date 2023-06-13
 * DANGER: 该段代码还没有在虚拟/实际的裁判系统上测试
 */
#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <nav_msgs/Odometry.h>
#include <robot_msg/MatchMsg.h>
#include <ros/ros.h>

namespace sp_decision {
class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  Blackboard() : match_state_received_(false), robot_odom_received_(false) {
    match_status_sub_ = nh_.subscribe("match_status", 1,
                                      &Blackboard::MatchStatusCallback, this);
    robot_odom_sub_ =
        nh_.subscribe("localization", 1, &Blackboard::RobotPoseCallback, this);
    nh_.param("min_hp", min_hp_, 200);
    nh_.param("min_bullet", min_bullet_, 100);
    nh_.param("min_outpost", min_outpost_, 300);
    nh_.param("distance_tolerance", distance_tolerance_, float(0.2));
  }
  ~Blackboard() {}
  std::mutex match_status_cbk_mutex;
  std::mutex robot_odom_cbk_mutex;

  void ResetFlag() {
    robot_odom_received_ = false;
    match_state_received_ = false;
  }

  /**
   * @brief 预先定义的属性
   */
  struct Point {
    double x;
    double y;
  };

  // buff点坐标: 原点、加血点、buff点
  std::vector<Point> buff_pos_ = {{0.0, 0.0}, {0.5, 2.79}, {1.9, 1.65}};
  int min_hp_;
  int min_bullet_;
  int min_outpost_;
  float distance_tolerance_;

  /**
   * @brief 从裁判系统读取的属性
   */
  bool match_state_;
  uint16_t robot_hp_;
  uint16_t robot_bullet_;
  uint16_t outpost_hp_;
  nav_msgs::Odometry robot_pose_;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber match_status_sub_;
  ros::Subscriber robot_odom_sub_;

  bool robot_odom_received_;
  bool match_state_received_;

  void MatchStatusCallback(const robot_msg::MatchMsg::ConstPtr msg) {
    match_status_cbk_mutex.lock();
    robot_hp_ = msg->robot_hp;
    robot_bullet_ = msg->robot_bullet;
    match_state_received_ = true;
    match_status_cbk_mutex.unlock();
  }

  void RobotPoseCallback(const nav_msgs::Odometry::ConstPtr msg) {
    robot_odom_cbk_mutex.lock();
    robot_pose_ = *msg;
    robot_odom_received_ = true;
    robot_odom_cbk_mutex.unlock();
  }
};
}  // namespace sp_decision
#endif