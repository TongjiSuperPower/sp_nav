#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <robot_msg/RobotStateMsg.h>

using namespace std;

namespace local_planner{
// PID控制器类
class PIDController {
private:
    double Kp_, Ki_, Kd_;  // PID参数
    double error_sum_, last_error_;  // 误差和、上一次误差
public:
    PIDController()
    {}
    PIDController(double Kp, double Ki, double Kd) 
    {
        Kp_ = Kp; 
        Ki_ = Ki;
        Kd_ = Kd;
        error_sum_ = 0; 
        last_error_ = 0;
    }

    // 计算输出
    double compute(double error, double dt) {
        error_sum_ += error * dt;
        double error_rate = (error - last_error_) / dt;
        last_error_ = error;
        return Kp_ * error + Ki_ * error_sum_ + Kd_ * error_rate;
    }

    // 重置控制器
    void reset() {
        error_sum_ = 0;
        last_error_ = 0;
    }
};

class LocalPlanner : public nav_core::BaseLocalPlanner{
public:

    LocalPlanner();
    LocalPlanner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~LocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;

    // 机器人状态相关
    enum RobotState {
        MOVE,
        CRUISR,
        IDLE,
        FAST,
        STOP,
    };
    RobotState state_;
    ros::Subscriber robot_state_sub_;

    bool initialized_, goal_reached_;
    double p_window_; // 下一个点的距离
    double p_precision_; // 目标点精度
    double max_v_; // 最大速度
    double max_vel_theta_; // 最大角速度

    double k_v_p_,k_v_i_,k_v_d_; // PID参数
    double d_t_;
    PIDController linear_distance_pid_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;

    std::string base_frame_;
    int plan_index_; // 跟踪点序号

    double x_, y_; // 当前位置

    geometry_msgs::PoseStamped target_ps_, current_ps_;

    // 避障有关参数
    double radius_; // 判断半径

    /**
     * @brief 转换位置到机体坐标系
     * @param src   初始位置
     * @param x     x
     * @param y     y
     */
    void getTransformedPosition(geometry_msgs::PoseStamped& src, double* x, double* y)
    {
        geometry_msgs::PoseStamped dst;
        src.header.stamp = ros::Time(0);
        tf_->transform(src, dst, base_frame_);
        *x = dst.pose.position.x;
        *y = dst.pose.position.y;
    }

    /**
     * @brief 获得到目标点距离
     * @param goal_ps 世界坐标系目标点坐标
     * @param x       世界坐标系当前x
     * @param y       世界坐标系当前y
     * @return the distance to the goal
     */
    double getGoalPositionDistance(const geometry_msgs::PoseStamped& goal_ps, double x, double y)
    {
        return std::hypot(x - goal_ps.pose.position.x, y - goal_ps.pose.position.y);
    }

    /**
     * @brief 判断机器人周围是否存在动态障碍物
     * @param costmap_ros   局部代价地图
     * @param radius 半径
     * @param goal_ps 世界坐标系下目标点
     * @return 是否有障碍物
     */
    bool isObstacle(const costmap_2d::Costmap2DROS* costmap_ros, double radius, geometry_msgs::PoseStamped goal_ps)
    {
        double goal_x = goal_ps.pose.position.x;
        double goal_y = goal_ps.pose.position.y;
        double resolution = costmap_ros_->getCostmap()->getResolution();
        bool has_obstacle = false;

        // 判断目标点范围内有没有障碍物
        for (int x = goal_x - radius; x <= goal_x + radius; x++) {
          for (int y = goal_y - radius; y <= goal_y + radius; y++) {
            unsigned int cell_x, cell_y;
            if (costmap_ros_->getCostmap()->worldToMap(x, y, cell_x, cell_y)) {
              unsigned char cost = costmap_ros_->getCostmap()->getCost(cell_x, cell_y);
              if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                has_obstacle = true;
                break;
              }
            }
          }
          if (has_obstacle) break;
        }
        if (has_obstacle) {
            return true;
        }
        else{
            return false;
        }

    }

    /**
     * @brief 订阅机器人当前状态，并更新
     * @param RobotState
     */
    void robotStateCallback(const robot_msg::RobotStateMsg::ConstPtr& msg)
    {
         // 从消息中读取枚举值
        int8_t robot_state_value = msg->robot_state;

        switch (robot_state_value) {
            case 0:
            state_ = RobotState::MOVE;
            break;
            case 1:
            state_ = RobotState::CRUISR;
            break;
            case 2:
            state_ = RobotState::IDLE;
            break;
            case 3:
            state_ = RobotState::FAST;
            break;
            case 4:
            state_ = RobotState::STOP;
            break;
            default:
            ROS_WARN("Invalid robot state value: %d", robot_state_value);
            return;
        }

        // 打印枚举值
        // ROS_INFO("Received robot state: %d", static_cast<int>(state_));
    }

    /**
     * @brief 根据不同策略计算速度
     * @param vel_output PID输出速度
     * @param theta 世界坐标系下航向角
     * @param cmd_vel 实际输出速度
     */
    void compute(double vel_output, double theta,geometry_msgs::Twist& cmd_vel)
    {
        
        // 不同策略速度不同
        double v(max_v_), ratio(1);
        switch (state_) {
            case RobotState::MOVE:
            cmd_vel.angular.z = 0;
            // 正常行驶速度
            v = max_v_;
            // 遇到障碍减速
            ratio = 0.8;
            break;
            case RobotState::CRUISR:
            cmd_vel.angular.z = 0.8 * max_vel_theta_;
            // 小陀螺行驶速度
            v = 0.1 * max_v_;
            // 遇到障碍减速
            ratio = 0.8;
            break;
            case RobotState::IDLE:
            cmd_vel.angular.z = max_vel_theta_;
            // 停止
            v = 0;
            ratio = 0;
            break;
            case RobotState::FAST:
            // 正常行驶速度
            v = max_v_ * 1.8;
            // 遇到障碍减速
            ratio = 0.8;
            break;
            case RobotState::STOP:
            // 正常行驶速度
            v = max_v_ * 0.0;
            // 遇到障碍减速
            ratio = 1.0;
            break;
            default:
            ROS_WARN("Invalid robot state value: %d", state_);
            return;
        }
        if (getGoalPositionDistance(global_plan_.back(), x_, y_) < p_precision_)
        {
            // 到终点
            vel_output = 0;
            linear_distance_pid_.reset();
            goal_reached_ = true;
        }
        else if(isObstacle(costmap_ros_, radius_, target_ps_))
        {
            // 目标点附近有障碍物，减速
            if (vel_output > ratio * v) {
                vel_output  = ratio * v;
            } else if (vel_output  < -ratio * v) {
                vel_output  = -ratio * v;
            }
        }
        else
        {
            // 限制线速度
            if (vel_output > v) {
                vel_output = v;
            } else if (vel_output < -v) {
                vel_output = -v;
            }
        }
        cmd_vel.linear.x = vel_output * cos(theta);
        cmd_vel.linear.y = vel_output * sin(theta);
    }
    
};
};

#endif



