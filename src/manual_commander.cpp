#include <ros/ros.h>

#include <one_unit_robocon_2022/Twist.h>

#include "CRSLib/state_manager.hpp"
#include "CRSLib/logicool.hpp"

#include "one_unit_robocon_2022/state.hpp"

using namespace CRSLib;
using namespace OneUnitRobocon2022;

namespace
{
    class ManualCommander
    {
        struct RosParamData
        {
            double pub_freq{};
            double max_body_linear_vel{};
            double max_body_angular_vel{};

            RosParamData() noexcept
            {
                ros::NodeHandle pnh{"~"};

                pnh.getParam("pub_freq", pub_freq);
                if(pub_freq <= 0)
                {
                    pub_freq = 1.0;
                    ROS_ERROR("Stew: rosparam error. pub_freq must be positive. pub_freq is set to the default value of 1.0.");
                }

                pnh.getParam("max_body_linear_vel", max_body_linear_vel);
                if(max_body_linear_vel <= 0)
                {
                    max_body_linear_vel = 1.0;
                    ROS_ERROR("Stew: rosparam error. max_body_linear_vel must be positive. max_body_linear_vel is set to the default value of 1.0.");
                }

                pnh.getParam("max_body_angular_vel", max_body_angular_vel);
                if(max_body_angular_vel <= 0)
                {
                    max_body_angular_vel = 1.0;
                    ROS_ERROR("Stew: rosparam error. max_body_angular_vel must be positive. max_body_angular_vel is set to the default value of 1.0.");
                }
            }
        } ros_param_data{};

        ros::NodeHandle nh{};
        ros::Publisher body_twist_pub{nh.advertise<one_unit_robocon_2022::Twist>("body_twist", 1)};

        StateManager<StateEnum, ManualCommander> state_manager{nh, this};
        ros::Timer pub_timer{};

        Logicool logicool{nh};

    public:
        ManualCommander() noexcept
        {
            pub_timer = nh.createTimer(ros::Duration(1 / ros_param_data.pub_freq), &ManualCommander::timerCallback, this);
        }

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
            if(logicool.is_pushed_once(Logicool::Buttons::start))
            {
                state_manager.set_state(OneUnitRobocon2022::StateEnum::reset);
            }
        }

        void case_reset() noexcept
        {
            if(logicool.is_pushed_once(Logicool::Buttons::start))
            {
                state_manager.set_state(OneUnitRobocon2022::StateEnum::manual);
            }
        }

        void case_manual() noexcept
        {
            if(logicool.is_pushed_once(Logicool::Buttons::start))
            {
                state_manager.set_state(OneUnitRobocon2022::StateEnum::disable);
            }

            one_unit_robocon_2022::Twist cmd_vel{};

            cmd_vel.linear_x = ros_param_data.max_body_linear_vel * logicool.axes[Logicool::Axes::l_stick_UD];
            cmd_vel.linear_y = ros_param_data.max_body_linear_vel * logicool.axes[Logicool::Axes::l_stick_LR];
            cmd_vel.angular_z = ros_param_data.max_body_angular_vel * logicool.axes[Logicool::Axes::r_stick_LR];

            // ROS_INFO("cmd_vel %lf, %lf", cmd_vel.linear_x, cmd_vel.linear_y);

            body_twist_pub.publish(cmd_vel);

        }

        void case_automatic() noexcept
        {
            if(logicool.is_pushed_once(Logicool::Buttons::start))
            {
                state_manager.set_state(OneUnitRobocon2022::StateEnum::manual);
            }
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_commander");

    ManualCommander manual_commander;
    
    ROS_INFO("manual_commander node has started.");
    
    ros::spin();
    
    ROS_INFO("manual_commander node has terminated.");

    return 0;
}