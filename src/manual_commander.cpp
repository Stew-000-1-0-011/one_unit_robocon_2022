#include "one_unit_robocon_2022/manual_commander.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_commander");

    ros::NodeHandle nh{};  // グローバルに似た名前空間であること(この名前空間上でcan通信やstate_managerなどのCRSLib内機能が動くため)。
    OneUnitRobocon2022::ManualCommander manual_commander{nh};
    
    ROS_INFO("Stew: manual_commander node has started.");
    
    ros::spin();
    
    ROS_INFO("Stew: manual_commander node has terminated.");

    return 0;
}