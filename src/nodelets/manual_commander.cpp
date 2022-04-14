#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "one_unit_robocon_2022/manual_commander.hpp"

namespace OneUnitRobocon2022
{
    namespace
    {
        class NodeletManualCommander final : public nodelet::Nodelet
        {
            // コンストラクタでgetNodeHandleを呼べるのか調べるのが面倒だったのでonInit内で初期化することにした。
            ManualCommander * manual_comander_dp{};

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                manual_comander_dp = new ManualCommander(nh);
            }

            ~NodeletManualCommander()
            {
                // C++14以降想定
                delete manual_comander_dp;
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(OneUnitRobocon2022::NodeletManualCommander, nodelet::Nodelet);
