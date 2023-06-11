//#ifndef AERIAL_NAVIGATION_H
//#define AERIAL_NAVIGATION_H
 
#include <actionlib/client/simple_action_client.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <robot_msg/RobotStateMsg.h>

//给MoveBaseAction定义一个别名，方便创建对象
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class GoalSending {
    private:
        #define max_vel_theta_ 8.0
        enum RobotState {
            MOVE,
            CRUISR,
            IDLE,
            FAST,
            STOP
        };

        Client Action;

        ros::NodeHandle nh_;
        ros::Subscriber localization_sub_;
        ros::Subscriber referee_data_sub_;
        ros::Publisher robot_state_pub_;
        // 原地小陀螺参数
        ros::Publisher vel_pub_;
        ros::Timer timer_pos_write_;

        struct Referee{
            geometry_msgs::Point referee_pos_;
            ros::Time stamp_;
        };
        realtime_tools::RealtimeBuffer<Referee> realtime_buffer_;

        bool localization_success_;

        geometry_msgs::PoseStamped target_pose_;
        move_base_msgs::MoveBaseGoal goal_;

        /**
        * @brief 监听定位状态
        * @param msg 定位消息
        */
        void localizationCallback(const std_msgs::Bool::ConstPtr& msg);
        /**
        * @brief 监听裁判系统返回的坐标
        * @param msg 定位消息
        */
        void refereeCallback(const geometry_msgs::Point::ConstPtr& msg);

        void posWrite(const ros::TimerEvent& event);

        void robotStatePub(RobotState state_);

        //判断是否接收到目标点
        void activeCb();
        void doneCb(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result);

        void idle();
        void stop();

    public:
        GoalSending();
};
