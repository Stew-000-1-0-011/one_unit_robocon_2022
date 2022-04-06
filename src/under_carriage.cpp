/*
base_controllerを参考にした。
機体に固定された座標での平面上移動におけるTwistを受け取り、各モーターへの速度をslcan_bridgeに向けてpublishしている。

どうせならURDFあたりから勝手に動くようにしたいね。
*/

#include <ros/ros.h>
#include "one_unit_robocon_2022/Twist.h"

#include "StewLib/Math/vec2d.hpp"

#include "CRSLib/shirasu.hpp"
#include "CRSLib/state_manager.hpp"
#include "CRSLib/rosparam_util.hpp"

#include "one_unit_robocon_2022/State.h"

using namespace StewLib::Math;
using namespace CRSLib;
using namespace OneUnitRobocon2022;

namespace
{
    class UnderCarriage final
    {
        ros::NodeHandle nh{};

        struct RosParamData
        {
            double pub_freq{};
            double max_wheel_vel{};
            double max_wheel_acc{};

            uint32_t FR_base_id{};
            uint32_t FL_base_id{};
            uint32_t BL_base_id{};
            uint32_t BR_base_id{};

            RosParamData() noexcept
            {
                ros::NodeHandle pnh{"~"};

                RosparamUtil::get_param(pnh, "max_wheel_vel", max_wheel_vel, RosparamUtil::is_positive, 1);

                pnh.getParam("max_wheel_vel", max_wheel_vel);
                if(max_wheel_vel <= 0)
                {
                    max_wheel_vel = 1.0;
                    ROS_ERROR("Stew: rosparam error. max_wheel_vel must be positive. max_wheel_vel is set to the default value of 1.0.");
                }

                pnh.getParam("max_wheel_acc", max_wheel_acc);
                if(max_wheel_acc <= 0)
                {
                    max_wheel_acc = 1.0;
                    ROS_ERROR("Stew: rosparam error. max_wheel_acc must be positive. max_wheel_acc is set to the default value of 1.0.");
                }

            }
        } ros_param_data{};

        ros::Timer publish_timer{};

        ros::Subscriber body_twist_sub{nh.subscribe<one_unit_robocon_2022::Twist>("body_twist", 1, &UnderCarriage::body_twist_callback, this)};

        StateManager<StateEnum, UnderCarriage> state_manager{nh, this};

        /// DEBUG:
        // Shirasu FR{nh, ros_param_data.FR_base_id};

        // Shirasu motors[4] =
        // {
        //     {nh, ros_param_data.FR_base_id},
        //     {nh, ros_param_data.FL_base_id},
        //     {nh, ros_param_data.BL_base_id},
        //     {nh, ros_param_data.BR_base_id}
        // };

        Vec2D<double> body_linear_vel{};
        double body_angular_vel{};

        double wheels_vela[4]{};
        double pre_wheels_vela[4]{};

        bool is_active{false};

    public:
        UnderCarriage() noexcept
        {
            publish_timer = nh.createTimer(ros::Duration(1 / ros_param_data.pub_freq), &UnderCarriage::publish_timer_callback, this);
        }

    private:
        void publish_timer_callback(const ros::TimerEvent&) noexcept
        {
            if(!is_active) return;

            calc_wheels_vela();

            /// DEBUG:
            // FR.send_target(wheels_vela[0]);

            // for(int i = 0; i < 4; ++i)
            // {
            //     motors[i].send_target(wheels_vela[i]);
            // }
        }

        void body_twist_callback(const one_unit_robocon_2022::Twist::ConstPtr& msg_p) noexcept
        {
        
            body_linear_vel = {msg_p->linear_x, msg_p->linear_y};
            body_angular_vel = msg_p->angular_z;
        }

        void state_manager_callback_inner(const StateEnum state) noexcept
        {
            switch(state)
            {
            case StateEnum::disable:
            case StateEnum::reset:
                is_active = false;
                break;
            case StateEnum::manual:
            case StateEnum::automatic:
                is_active = true;
            }
        }
        
        inline void calc_wheels_vela() noexcept
        {
            // using namespace Config::Wheel;

            // constexpr double rot_factors[4] =
            // {
            //     Config::body_radius * rot(~Pos::FR,Constant::PI_2) * ~Direction::FR,
            //     Config::body_radius * rot(~Pos::FL,Constant::PI_2) * ~Direction::FL,
            //     Config::body_radius * rot(~Pos::BL,Constant::PI_2) * ~Direction::BL,
            //     Config::body_radius * rot(~Pos::BR,Constant::PI_2) * ~Direction::BR
            // };

            // const auto body_linear_vel = this->body_linear_vel;
            // const auto body_angular_vel = this->body_angular_vel;
            // double wheels_vela[4];

            // for(int i = 0; i < 4; ++i)
            // {
            //     wheels_vela[i] = (~Direction::all[i] * body_linear_vel + rot_factors[i] * body_angular_vel) / Config::wheel_radius;
            // }

            // if constexpr (Config::Limitation::wheel_acca)
            // {
            //     double diffs_vela[4];
            //     for(int i = 0; i < 4; ++i)
            //     {
            //         diffs_vela[i] = pre_wheels_vela[i] - wheels_vela[i];
            //     }

            //     auto max = diffs_vela[0];
            //     for(int i = 1; i < 4; ++i)
            //     {
            //         if(max < diffs_vela[i]) max = diffs_vela[i];
            //     }
                
            //     if(max > Config::Limitation::wheel_acca)
            //     {
            //         ROS_WARN("%s: warning: The accelaretion of the wheels is too high. Speed is limited.", StringlikeTypes::under_carriage_4wheel::str);
            //         auto limit_factor = Config::Limitation::wheel_acca / max;
            //         for(int i = 0; i < 4; ++i)
            //         {
            //             wheels_vela[i] += pre_wheels_vela[i] + (limit_factor * diffs_vela[i]);
            //         }
            //     }

            //     for(int i = 0; i < 4; ++i)
            //     {
            //         this->wheels_vela[i] = wheels_vela[i];
            //     }
            // }

            // if constexpr (Config::Limitation::wheel_vela)
            // {
            //     auto max = wheels_vela[0];
            //     for(int i = 1; i < 4; ++i)
            //     {
            //         if(max < wheels_vela[i]) max = wheels_vela[i];
            //     }
                
            //     if(max > Config::Limitation::wheel_vela)
            //     {
            //         ROS_WARN("%s: warning: The speed of the wheels is too high. Speed is limited.", StringlikeTypes::under_carriage_4wheel::str);
            //         auto limit_factor = Config::Limitation::wheel_vela / max;
            //         for(int i = 0; i < 4; ++i)
            //         {
            //             wheels_vela[i] *= limit_factor;
            //         }
            //     }
            // }

            // if constexpr(Config::Limitation::wheel_acca)
            // {
            //     for(int i = 0; i < 4; ++i)
            //     {
            //         pre_wheels_vela[i] = wheels_vela[i];
            //     }
            // }
        }
    };
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "under_carriage");

    UnderCarriage under_carriage;

    ROS_INFO("under_carriage node has started.");

    ros::spin();

    ROS_INFO("under_carriage node has terminated.");

}

