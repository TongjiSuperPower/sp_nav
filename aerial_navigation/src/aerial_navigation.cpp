#include "aerial_navigation/aerial_navigation.h"

GoalSending::GoalSending() : Action("move_base", true) {

    // 订阅定位是否成功
    localization_sub_ = nh_.subscribe("/localization_success", 1, &GoalSending::localizationCallback, this);
    // 发布机器人状态
    robot_state_pub_ = nh_.advertise<robot_msg::RobotStateMsg>("robot_state", 1);
    // 发布静止时的机器人速度
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // 订阅从裁判系统发过来的的坐标位置
    referee_data_sub_ = nh_.subscribe("/referee_data", 1, &GoalSending::refereeCallback, this);
    // 构建机器人位置发布定时器
    timer_pos_write_ = nh_.createTimer(ros::Duration(0.1), &GoalSending::posWrite, this);

    ROS_INFO("Starting success");

}

void GoalSending::localizationCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data)
    {
        localization_success_ = true;
    }
}

void GoalSending::idle() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = max_vel_theta_;
    vel_pub_.publish(cmd_vel);
}

void GoalSending::stop() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    vel_pub_.publish(cmd_vel);
}

void GoalSending::refereeCallback(const geometry_msgs::Point::ConstPtr& msg) {
    //使用realtime_buffer保证接收
    Referee referee_struct_temp{ .referee_pos_ = *msg, .stamp_ = ros::Time::now() };
    realtime_buffer_.writeFromNonRT(referee_struct_temp);
}

void GoalSending::robotStatePub(RobotState state_) {
    robot_msg::RobotStateMsg robot_state_msg;
    robot_state_msg.robot_state = static_cast<int8_t>(state_);
    robot_state_pub_.publish(robot_state_msg);
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

void GoalSending::posWrite(const ros::TimerEvent& event) {
    const geometry_msgs::Point& referee_pos = realtime_buffer_.readFromNonRT()->referee_pos_;

    if (!Action.waitForServer(ros::Duration(60)))
    {
        ROS_INFO("Can't connected to move base server");
    }
    //判断相同点不执行
    if (target_pose_.pose.position.x == referee_pos.x && target_pose_.pose.position.y == referee_pos.y) return;
    //原地小陀螺
    if (fabs(referee_pos.z - 's') < 1e-3 || fabs(referee_pos.z - 'S') < 1e-3)
    {
        robotStatePub(IDLE);
        idle();
        return;
    }
    if (fabs(referee_pos.z - 'q') < 1e-3 || fabs(referee_pos.z - 'Q') < 1e-3)
    {
        robotStatePub(STOP);
        stop();
        return;
    }


    target_pose_.header.frame_id = "world";
    target_pose_.header.stamp = ros::Time::now();

    target_pose_.pose.position.x = referee_pos.x;
    target_pose_.pose.position.y = referee_pos.y;
    target_pose_.pose.orientation.x = 0.0;
    target_pose_.pose.orientation.y = 0.0;
    target_pose_.pose.orientation.z = 1.0;
    target_pose_.pose.orientation.w = 0.0;

    goal_.target_pose = target_pose_;


    if (fabs(referee_pos.z - 'd') < 1e-3 || fabs(referee_pos.z - 'D') < 1e-3)
    {
        //小陀螺前进
        robotStatePub(CRUISR);
    }
    else if (fabs(referee_pos.z - 'a') < 1e-3 || fabs(referee_pos.z - 'A') < 1e-3)
    {
        //快速前进
        robotStatePub(FAST);
    }
    else
    {
        //普通速度前进
        robotStatePub(MOVE);
    }
    Action.sendGoal(goal_, boost::bind(&GoalSending::doneCb, this, _1, _2),
        boost::bind(&GoalSending::activeCb, this),
        Client::SimpleFeedbackCallback());

    //Action.waitForResult();
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goalsending");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    GoalSending gs;
    ros::waitForShutdown();
    return 0;
}
