#include <memory>

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
            std::unique_ptr<ManualCommander> manual_comander_up{};

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                manual_comander_up = std::make_unique<ManualCommander>(nh);
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(OneUnitRobocon2022::NodeletManualCommander, nodelet::Nodelet)
