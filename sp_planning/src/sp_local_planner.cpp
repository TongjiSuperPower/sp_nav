#include "sp_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

namespace local_planner{

LocalPlanner::LocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

LocalPlanner::LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;

        ros::NodeHandle nh = ros::NodeHandle("~/" + name);
        robot_state_sub_ = nh.subscribe("/robot_state", 1, &LocalPlanner::robotStateCallback,this);

        nh.param<double>("/tracking_tolerance", p_window_, 0.5);
        nh.param<double>("/distance_tolerance", p_precision_, 0.2);
        nh.param<double>("/max_linear_speed", max_v_, 0.8);
        nh.param<double>("/max_vel_theta", max_vel_theta_, 3.14);
        nh.param<double>("/radius", radius_, 0.2);

        nh.param<double>("/linear_kp", k_v_p_, 1.50);
        nh.param<double>("/linear_ki", k_v_i_, 0.01);
        nh.param<double>("/linear_kd", k_v_d_, 0.01);

        base_frame_ = "base_link";

        double controller_freqency;
        nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
        d_t_ = 1 / controller_freqency;
        // linear_distance_pid_ = PIDController(k_v_p_, k_v_i_, k_v_d_);
        ROS_INFO("PID planner initialized!");
        
        state_ = RobotState::MOVE;
    }
    else
    {
        ROS_WARN("PID planner has already been initialized.");
    }
}

/**
 * @brief  更新全局路径
 * @param orig_global_plan move_base全局路径
 * @return  是否更新成功
 */
bool LocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    ROS_INFO("Got new plan");

    // 更新全局路径
    global_plan_ = orig_global_plan;

    // 调整跟踪点
    plan_index_ = 3;  
    goal_reached_ = false;

    return true;
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    if (goal_reached_)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        // linear_distance_pid_.reset();
        return true;
    }
    costmap_ros_->getRobotPose(current_ps_);
    x_ = current_ps_.pose.position.x;
    y_ = current_ps_.pose.position.y;
    
    // 机体坐标系下目标点
    double b_x_d, b_y_d;

    while (plan_index_ < global_plan_.size())
    {
        target_ps_ = global_plan_[plan_index_];
        // TODO：优化减速点检测
        /*
        double n = global_plan_.size() - plan_index_;
        if (n > 5){
            slow_ps_ = global_plan_[plan_index_ + 5];
        }
        else{
            slow_ps_ = global_plan_[plan_index_ + n - 1];
        }
        ROS_INFO_STREAM("target_ps_" << target_ps_.pose.position.x << " " << target_ps_.pose.position.y); 
        ROS_INFO_STREAM("slow_ps_" << slow_ps_.pose.position.x << " " << slow_ps_.pose.position.y); 
        */
        getTransformedPosition(target_ps_, &b_x_d, &b_y_d);
        if (std::hypot(b_x_d, b_y_d) > p_window_ )
            break;
        plan_index_++;
    }
    double distance = std::hypot(b_x_d, b_y_d);
    // double vel_output = linear_distance_pid_.compute(distance, d_t_);
    double vel_output = k_v_p_ * distance;
    double theta = atan2(b_y_d, b_x_d);
    compute(vel_output, theta, cmd_vel);
    return true;
}

bool LocalPlanner::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    if (!costmap_ros_->getRobotPose(current_ps_))
    {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    if (goal_reached_)
    {
        ROS_INFO("Goal reached");
        // linear_distance_pid_.reset();
        return true;
    }

    if (plan_index_ > global_plan_.size() - 1)
    {
        // if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)
        // {
        //     goal_reached_ = true;
        //     linear_distance_pid_.reset();
        //     ROS_INFO("Goal reached");
        // }
        // linear_distance_pid_.reset();
        ROS_INFO("Goal reached");
        goal_reached_ = true;
    }
    return goal_reached_;
}
}