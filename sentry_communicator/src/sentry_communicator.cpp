#include "sentry_communicator/can_bus.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_communicator");
    ros::NodeHandle nh;

    sentry_communicator::CanBus can_bus("can0", 20, nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while(ros::ok())
    {
        can_bus.write();
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
    ros::waitForShutdown();
}
