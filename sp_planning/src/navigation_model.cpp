#include <navigation_model.h>

GoalSending::GoalSending() : ac_("move_base", true), cruise_initialized_(false) {
  // 读取目标点
  this->pointRead();

  // 订阅定位是否成功
  localization_sub_ = nh_.subscribe("/localization_success", 1, &GoalSending::localizationCallback,this);

  // 发布机器人状态
  robot_state_pub_ = nh_.advertise<robot_msg::RobotStateMsg>("robot_state", 1);

  // 发布静止时的机器人速度
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // 机器人状态初始化为执行序列
  this->setState(RobotState::OUTPOST);

  // 原地小陀螺不是最后状态
  end_goal_ = false;

  // 构建机器人状态发布定时器
  timer_robot_state_ = nh_.createTimer(ros::Duration(0.3), &GoalSending::robotStatePub, this);

  // 构建状态监测定时器
  timer_state_check_ =
      nh_.createTimer(ros::Duration(0.1), &GoalSending::stateCheck, this);  
  ROS_INFO("Starting success");
}

void GoalSending::localizationCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
    	localization_success_ = true;
    }
}


void GoalSending::robotStatePub(const ros::TimerEvent& event) {
  robot_msg::RobotStateMsg robot_state_msg;
  robot_state_msg.robot_state = static_cast<int8_t>(state_);
  robot_state_pub_.publish(robot_state_msg);
}


void GoalSending::setState(RobotState state) {
  state_ = state;
}


void GoalSending::pointRead() {
  std::vector<double> tmp {0.0, 0.0, 1.0, 0.0};
  nh_.param<std::vector<double>>("/navigation_model/two_point_cruise/first_goal",first_goal_,tmp);
  nh_.param<std::vector<double>>("/navigation_model/two_point_cruise/second_goal",second_goal_,tmp);
  nh_.param<double>("/max_vel_theta",max_vel_theta_,3.14);
  
  XmlRpc::XmlRpcValue outpost_list;
  nh_.getParam("/navigation_model/outpost_goal", outpost_list);
  if (outpost_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < outpost_list.size(); ++i) {
      XmlRpc::XmlRpcValue xmlrpc_element = outpost_list[i];
      if (xmlrpc_element.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        std::vector<double> element;
        for (int j = 0; j < xmlrpc_element.size(); ++j) {
          double value = static_cast<double>(xmlrpc_element[j]);
          element.push_back(value);
        }
        outpost_goal_.push_back(element);
      }
    }
  }
  XmlRpc::XmlRpcValue defence_list;
  nh_.getParam("/navigation_model/defence_goal", defence_list);
  if (defence_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < defence_list.size(); ++i) {
      XmlRpc::XmlRpcValue xmlrpc_element = defence_list[i];
      if (xmlrpc_element.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        std::vector<double> element;
        for (int j = 0; j < xmlrpc_element.size(); ++j) {
          double value = static_cast<double>(xmlrpc_element[j]);
          element.push_back(value);
        }
        defence_goal_.push_back(element);
      }
    }
  }

  XmlRpc::XmlRpcValue outpost_next_list;
  nh_.getParam("/navigation_model/outpost_next", outpost_next_list);
  if (outpost_next_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < outpost_next_list.size(); ++i) {
      XmlRpc::XmlRpcValue xmlrpc_element = outpost_next_list[i];
      if (xmlrpc_element.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        std::vector<double> element;
        for (int j = 0; j < xmlrpc_element.size(); ++j) {
          double value = static_cast<double>(xmlrpc_element[j]);
          element.push_back(value);
        }
        outpost_next_goal_.push_back(element);
      }
    }
  }
  
  nh_.param<double>("/navigation_model/first_duration",first_duration_,10);
  nh_.param<double>("/navigation_model/second_duration",second_duration_,10);
}

void GoalSending::doneCb(const actionlib::SimpleClientGoalState& state,
                         const move_base_msgs::MoveBaseResultConstPtr& result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("SUCCEEDED");
  }
}

void GoalSending::activeCb() {
  ROS_INFO("Goal Received");
}

void GoalSending::stateCheck(const ros::TimerEvent& event) {
  switch (state_) {
    case RobotState::OUTPOST:
    popSequential(outpost_goal_,RobotState::OUTPOST,RobotState::IDLE); 
    break;
    case RobotState::CRUISR:
    twoPointCruise();
    break;
    case RobotState::IDLE:
    if(!end_goal_)
    {
      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - idle_start_;
      if (elapsed_time.toSec() >= duration_)
      {
        this->setState(next_state_);
        break;
      }
    }
    idle();
    break;
    case RobotState::DEFENCE:
    popSequential(defence_goal_,RobotState::DEFENCE,RobotState::IDLE); 
    break;
    case RobotState::OUTPOST_NEXT:
    popSequential(outpost_next_goal_,RobotState::OUTPOST_NEXT,RobotState::IDLE); 
    break;
    default:
    ROS_WARN("Invalid robot state value: %d", state_);
    return;
  }
}

void GoalSending::idle(){
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = max_vel_theta_;
      vel_pub_.publish(cmd_vel);
}

void GoalSending::popSequential(std::deque<std::vector<double>> &goal_list, RobotState cur_state,RobotState next_state){
    //遍历所有的目标点
      if(!goal_list.empty()) 
      {
        std::vector<double> tmp_goal = goal_list.front(); 
        target_pose_.header.frame_id = "world";
        target_pose_.header.stamp = ros::Time::now();

        target_pose_.pose.position.x = tmp_goal[0];
        target_pose_.pose.position.y = tmp_goal[1];
        target_pose_.pose.orientation.x = 0.0;
        target_pose_.pose.orientation.y = 0.0;
        target_pose_.pose.orientation.z = tmp_goal[2];
        target_pose_.pose.orientation.w = tmp_goal[3];

        goal_.target_pose = target_pose_;

        if (!ac_.waitForServer(ros::Duration(60))) {
          ROS_INFO("Can't connected to move base server");
        }

        ac_.sendGoal(goal_, boost::bind(&GoalSending::doneCb, this, _1, _2),
                  boost::bind(&GoalSending::activeCb, this),
                  Client::SimpleFeedbackCallback());

        ac_.waitForResult();

        if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          // 前往下一个目标点
          goal_list.pop_front();
        }
        else{
          idle_start_ = ros::Time::now();
          end_goal_ = false;
          duration_ = 20;
          next_state_ = cur_state;
          this->setState(RobotState::IDLE);
        }
      }
      // 切换机器人状态
      else{
        if(cur_state == RobotState::OUTPOST && next_state == RobotState::IDLE){
          idle_start_ = ros::Time::now();
          next_state_ = RobotState::DEFENCE;
          end_goal_ = false;
          duration_ = first_duration_;
        }
        if(cur_state == RobotState::DEFENCE && next_state == RobotState::IDLE){
          idle_start_ = ros::Time::now();
          next_state_ = RobotState::OUTPOST_NEXT;
          end_goal_ = false;
          duration_ = second_duration_;
        }
        if(cur_state == RobotState::OUTPOST_NEXT && next_state == RobotState::IDLE){
          idle_start_ = ros::Time::now();
          end_goal_ = true;
          duration_ = 0.0;
        }
        this->setState(next_state);
      }
}

void GoalSending::twoPointCruise() {
  //发送初始点位
  if (!cruise_initialized_) {
    target_pose_.header.seq = 1;
    target_pose_.header.frame_id = "world";
    target_pose_.header.stamp = ros::Time::now();
    target_pose_.pose.position.x = first_goal_[0];
    target_pose_.pose.position.y = first_goal_[1];
    target_pose_.pose.orientation.x = 0.0;
    target_pose_.pose.orientation.y = 0.0;
    target_pose_.pose.orientation.z = first_goal_[2];
    target_pose_.pose.orientation.w = first_goal_[3];
    goal_.target_pose = target_pose_;
    cruise_initialized_ = true;
    if (!ac_.waitForServer(ros::Duration(60))) {
      ROS_INFO("Can't connected to move base server");
    }
    ac_.sendGoal(goal_, boost::bind(&GoalSending::doneCb, this, _1, _2),
                boost::bind(&GoalSending::activeCb, this),
                Client::SimpleFeedbackCallback());     
  }
  //判断是否到达目标点，如果成功，则发布下一个点
  if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    count_++;
    target_pose_.header.seq = count_ + 1;
    target_pose_.header.frame_id = "world";
    target_pose_.header.stamp = ros::Time::now();
    target_pose_.pose.position.x = second_goal_[0];
    target_pose_.pose.position.y = second_goal_[1];
    target_pose_.pose.orientation.x = 0.0;
    target_pose_.pose.orientation.y = 0.0;
    target_pose_.pose.orientation.z = second_goal_[2];
    target_pose_.pose.orientation.w = second_goal_[3];
    goal_.target_pose = target_pose_;
    if (!ac_.waitForServer(ros::Duration(60))) {
      ROS_INFO("Can't connected to move base server");
    }
    ac_.sendGoal(goal_, boost::bind(&GoalSending::doneCb, this, _1, _2),
                boost::bind(&GoalSending::activeCb, this),
                Client::SimpleFeedbackCallback());
  }
  // 如果发送失败，则不断尝试
  if (ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) 
  {
    // count_ = 1说明成功到达第一个点，count_ = 2说明成功到达第二个点
    ac_.sendGoal(goal_, boost::bind(&GoalSending::doneCb, this, _1, _2),
        boost::bind(&GoalSending::activeCb, this),
        Client::SimpleFeedbackCallback());  
  }
  // 如果已经到达第二个点，则初始化，回到第一个点，循环运动
  if (count_ == 2) {
    cruise_initialized_ = false;
    count_ = 0;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "goalsending");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  GoalSending gs;
  ros::waitForShutdown();
  return 0;
}


