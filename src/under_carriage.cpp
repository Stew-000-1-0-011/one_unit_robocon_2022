#include "one_unit_robocon_2022/under_carriage.hpp"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "under_carriage");
    ros::NodeHandle nh{};  // グローバルに似た名前空間であること。

    OneUnitRobocon2022::UnderCarriage under_carriage{nh};

    ROS_INFO("Stew: under_carriage node has started.");

    ros::spin();

    ROS_INFO("Stew: under_carriage node has terminated.");

}

