// #include "one_unit_robocon_2022/manual_commander.hpp"

// using namespace CRSLib::RosparamUtil::Implement;

// static_assert(is_string_like<const int>);

#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hogehogehoge");
    ros::Publisher pub{};
    ROS_INFO("%d", static_cast<bool>(pub));
    pub = ros::NodeHandle().advertise<std_msgs::Bool>("aaa", 1);
    ROS_INFO("%d", static_cast<bool>(pub));
}