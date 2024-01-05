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
#include <actionlib_msgs/GoalStatusArray.h>
/**0：PENDING（等待） - 导航目标正在等待处理。

1：ACTIVE（激活） - 导航目标正在被执行。

2：PREEMPTED（中断） - 导航目标被中断。

3：SUCCEEDED（成功完成） - 导航目标成功完成。

4：ABORTED（中止） - 导航目标被中止。

5：REJECTED（拒绝） - 导航目标被拒绝。

6：PREEMPTING（正在中断） - 导航目标正在被中断。

7：RECALLING（正在召回） - 导航目标正在被召回。

8：RECALLED（已召回） - 导航目标已经被召回。

9：LOST（丢失） - 导航目标丢失。
*/
namespace sp_decision {
enum class NavState { ACTIVE, PREEMPTED, SUCCEEDED, ABORTED, REJECTED, PREEMPTING, RECALLING, RECALLED, LOST };
class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  Blackboard() : match_state_received_(false), robot_odom_received_(false) {
    match_status_sub_ = nh_.subscribe("match_status", 1,
                                      &Blackboard::MatchStatusCallback, this);
    robot_odom_sub_ =
        nh_.subscribe("localization", 1, &Blackboard::RobotPoseCallback, this);
        
    referee_data_sub_ = nh_.subscribe("referee_data", 1, 
                                        &Blackboard::RefereeDataCallback, this);
    move_base_status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status",10000,
                                         &Blackboard::MoveBaseStatusCallback, this);

                                        
    nh_.param("min_hp", min_hp_, 200);
    nh_.param("min_bullet", min_bullet_, 100);
    nh_.param("min_outpost", min_outpost_, 300);
    nh_.param("distance_tolerance", distance_tolerance_, float(0.2));
  }
  ~Blackboard() {}
  std::mutex match_status_cbk_mutex;
  std::mutex robot_odom_cbk_mutex;
  std::mutex referee_data_cbk_mutex;
  std::mutex move_base_status_cbk_mutex;

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
  std::vector<Point> buff_pos_ = {{0.0, 0.0}, {2.5, 0}, {-0.5, 0}};
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

  /**
   * @brief 云台手通信
   */
  float posx_x_;
  float posy_y_;
  float key_z_;
  /**
   * @brief 机器人导航状态
   */
   NavState nav_state_;


 private:
  ros::NodeHandle nh_;
  ros::Subscriber match_status_sub_;
  ros::Subscriber robot_odom_sub_;
  ros::Subscriber referee_data_sub_;
  ros::Subscriber move_base_status_sub_;

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
  void RefereeDataCallback(const geometry_msgs::Point::ConstPtr& msg){
    referee_data_cbk_mutex.lock();
    posx_x_ = msg->x;
    posy_y_ = msg->y;
    key_z_  = msg->z; 
    referee_data_cbk_mutex.unlock();
  }
  void MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    move_base_status_cbk_mutex.lock();
    std::cout<<msg->status_list[0].text<<std::endl;
    if(msg->status_list[0].status == 1)
    {
      nav_state_ = NavState::ACTIVE;
    }
    else if(msg->status_list[0].status == 3)
    {
      nav_state_ = NavState::SUCCEEDED;
    }
    move_base_status_cbk_mutex.unlock();
  }
};
}  // namespace sp_decision
#endif