#include <numbers>

#include <ros/ros.h>

#include <one_unit_robocon_2022/Twist.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "CRSLib/state_manager.hpp"
#include "CRSLib/logicool.hpp"
#include "CRSLib/rosparam_util.hpp"

#include "one_unit_robocon_2022/state.hpp"

namespace OneUnitRobocon2022
{
    namespace
    {
        class ManualCommander final
        {
            struct RosParamData
            {
                double control_freq{};
                double max_body_linear_vel{};
                double max_body_angular_vel{};

                RosParamData() noexcept
                {
                    using namespace CRSLib::RosparamUtil;

                    ros::NodeHandle pnh{"~"};
                    std::optional<XmlRpc::XmlRpcValue> manual_commander_opt{XmlRpc::XmlRpcValue()};
                    pnh.getParam("manual_commander", *manual_commander_opt);

                    control_freq = read_param<double>(manual_commander_opt, "control_freq");
                    assert_param(control_freq, is_positive, 1000);

                    max_body_linear_vel = read_param<double>(manual_commander_opt, "max_body_linear_vel");
                    assert_param(max_body_linear_vel, is_positive, 2 * std::numbers::pi * 1);

                    max_body_angular_vel = read_param<double>(manual_commander_opt, "max_body_angular_vel");
                    assert_param(max_body_angular_vel, is_positive, 2 * std::numbers::pi * 0.5);
                }
            } ros_param_data{};

            ros::Publisher body_twist_pub;

            CRSLib::StateManager<StateEnum, ManualCommander> state_manager;
            ros::Timer pub_timer;

            CRSLib::Logicool logicool;

        public:
            ManualCommander(ros::NodeHandle& nh) noexcept:
                body_twist_pub{nh.advertise<one_unit_robocon_2022::Twist>("body_twist", 1)},
                state_manager{nh, this},
                pub_timer{nh.createTimer(ros::Duration(1 / ros_param_data.control_freq), &ManualCommander::timerCallback, this)},
                logicool{nh}
            {}

        private:
            void timerCallback(const ros::TimerEvent&) noexcept
            {
                switch(state_manager.get_state())
                {
                case OneUnitRobocon2022::StateEnum::disable:
                    case_disable();
                    break;

                case OneUnitRobocon2022::StateEnum::reset:
                    case_reset();
                    break;

                case OneUnitRobocon2022::StateEnum::manual:
                    case_manual();
                    break;
                
                case OneUnitRobocon2022::StateEnum::automatic:
                    case_automatic();
                    break;
                }
            }

            void case_disable() noexcept
            {
                if(logicool.is_pushed_once(CRSLib::Logicool::Buttons::start))
                {
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::reset);
                }
            }

            void case_reset() noexcept
            {
                if(logicool.is_pushed_once(CRSLib::Logicool::Buttons::start))
                {
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::manual);
                }
            }

            void case_manual() noexcept
            {
                if(logicool.is_pushed_once(CRSLib::Logicool::Buttons::start))
                {
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::disable);
                }

                one_unit_robocon_2022::Twist cmd_vel{};

                cmd_vel.linear_x = ros_param_data.max_body_linear_vel * logicool.axes[CRSLib::Logicool::Axes::l_stick_UD];
                cmd_vel.linear_y = ros_param_data.max_body_linear_vel * logicool.axes[CRSLib::Logicool::Axes::l_stick_LR];
                cmd_vel.angular_z = ros_param_data.max_body_angular_vel * logicool.axes[CRSLib::Logicool::Axes::r_stick_LR];

                body_twist_pub.publish(cmd_vel);

            }

            void case_automatic() noexcept
            {
                if(logicool.is_pushed_once(CRSLib::Logicool::Buttons::start))
                {
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::manual);
                }
            }
        };

        class NodeletManualCommander final : public nodelet::Nodelet
        {
            // コンストラクタでgetNodeHandleを呼べるのか調べるのが面倒だったのでonInit内で初期化することにした。
            ManualCommander * manual_comander_dp{};

            void onInit()
            {
                ros::NodeHandle nh = getNodeHandle();
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

