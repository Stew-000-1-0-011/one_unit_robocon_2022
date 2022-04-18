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
            UnderCarriage * under_carriage_dp{};

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                under_carriage_dp = new UnderCarriage(nh);
            }

            ~NodeletUnderCarriage()
            {
                // C++14以降想定
                delete under_carriage_dp;
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(OneUnitRobocon2022::NodeletUnderCarriage, nodelet::Nodelet)
