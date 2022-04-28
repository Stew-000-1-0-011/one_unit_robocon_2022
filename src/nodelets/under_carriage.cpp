#include <ros/ros.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "one_unit_robocon_2022/under_carriage.hpp"

namespace OneUnitRobocon2022
{
    namespace
    {
        class NodeletUnderCarriage final : public nodelet::Nodelet
        {
            std::unique_ptr<UnderCarriage> under_carriage_up{};

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                under_carriage_up = std::make_unique<UnderCarriage>(nh);
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(OneUnitRobocon2022::NodeletUnderCarriage, nodelet::Nodelet)
