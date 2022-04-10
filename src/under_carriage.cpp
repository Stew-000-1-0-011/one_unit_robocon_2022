/*
base_controllerを参考にした。
機体に固定された座標での平面上移動におけるTwistを受け取り、各モーターへの速度をslcan_bridgeに向けてpublishしている。

どうせならURDFあたりから勝手にタイヤの位置情報とか読み出せるようにしたい。
*/

// 標準ライブラリ
#include <numbers>

// ROSライブラリ((ほぼ)自動生成のものを含む)
#include <ros/ros.h>
#include "one_unit_robocon_2022/Twist.h"

// StewLibライブラリ(拙作で本当につたないのでダメ出し待ってます)(C++20以上に対応(concepts無くてもギリ動くぐらい))
#include "StewLib/Math/vec2d.hpp"

// CRSLibライブラリ(同上。)
#include "CRSLib/shirasu.hpp"
#include "CRSLib/state_manager.hpp"
#include "CRSLib/rosparam_util.hpp"

// 今回だけ使うヘッダ
#include "one_unit_robocon_2022/State.h"

using namespace StewLib::Math;
using namespace CRSLib;
using namespace OneUnitRobocon2022;

namespace
{
    struct Wheel final
    {
        uint32_t base_id{};
        Vec2D<double> position{};
        Vec2D<double> direction{};

        Wheel() = default;
        Wheel& operator=(Wheel&&) = default;

        Wheel(XmlRpc::XmlRpcValue& wheel_list) noexcept
        {
            using namespace RosparamUtil;

            base_id = read_param<int>(wheel_list, "base_id");
            auto position_list = get_param(wheel_list, "position");
            position = {read_param<double>(position_list, 0), read_param<double>(position_list, 1)};
            auto direction_list = get_param(wheel_list, "direction");
            direction = {read_param<double>(direction_list, 0), read_param<double>(direction_list, 1)};
        }
    };

    class UnderCarriage final
    {
        friend struct StateManagerCallback<UnderCarriage>;

        ros::NodeHandle nh{};

        // rosparamを使って初期化したいconstな非静的メンバ変数をまとめて初期化するためにクラスで包んでいる
        const struct RosParamData final
        {
            double pub_freq{};
            double max_wheel_vel{};
            double max_wheel_acc{};

            /// TODO: N個のタイヤに対応
            // std::vector<Wheel> wheels = std::vector<wheel>(n);
            Wheel wheels[4]{};

            RosParamData() noexcept
            {
                using CRSLib::RosparamUtil::assert_param;
                using CRSLib::RosparamUtil::get_param;
                using CRSLib::RosparamUtil::xml_rpc_cast;
                using CRSLib::RosparamUtil::read_param;
                using CRSLib::RosparamUtil::is_positive;
                using CRSLib::RosparamUtil::is_not_negative;
                using CRSLib::RosparamUtil::is_nonzero;

                ros::NodeHandle pnh{"~"};
                std::optional<XmlRpc::XmlRpcValue> under_carriage_opt{XmlRpc::XmlRpcValue()};
                pnh.getParam("under_carriage", *under_carriage_opt);

                pub_freq = read_param<double>(under_carriage_opt, "pub_freq");
                assert_param(pub_freq, is_positive, 1000);

                max_wheel_vel = read_param<double>(under_carriage_opt, "max_wheel_vel");
                assert_param(max_wheel_vel, is_not_negative, 0);

                max_wheel_acc = read_param<double>(under_carriage_opt, "max_wheel_acc");
                assert_param(max_wheel_acc, is_not_negative, 0);

                auto wheels_opt = get_param(under_carriage_opt, "wheels");
                for(int i = 0; i < 4; ++i) if(auto wheel_opt = get_param(wheels_opt, i); wheel_opt.has_value())
                {
                    wheels[i] = *wheel_opt;
                }
            }
        } ros_param_data{};

        ros::Timer publish_timer{};

        ros::Subscriber body_twist_sub{nh.subscribe<one_unit_robocon_2022::Twist>("body_twist", 1, &UnderCarriage::body_twist_callback, this)};

        StateManager<StateEnum, UnderCarriage> state_manager{this};

        Shirasu motors[4] =
        {
            {ros_param_data.wheels[0].base_id},
            {ros_param_data.wheels[1].base_id},
            {ros_param_data.wheels[2].base_id},
            {ros_param_data.wheels[3].base_id}
        };

        Vec2D<double> body_linear_vel{};
        double body_angular_vel{};

        double wheels_vel[4]{};
        double pre_wheels_vel[4]{};

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

            for(int i = 0; i < 4; ++i)
            {
                motors[i].send_target(wheels_vel[i]);
            }
        }

        void body_twist_callback(const one_unit_robocon_2022::Twist::ConstPtr& msg_p) noexcept
        {
        
            body_linear_vel = {msg_p->linear_x, msg_p->linear_y};
            body_angular_vel = msg_p->angular_z;
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

namespace CRSLib
{
    namespace
    {
        template<>
        struct StateManagerCallback<UnderCarriage>
        {
            void callback(UnderCarriage *const this_p, const StateEnum state) noexcept
            {
                switch(state)
                {
                case StateEnum::disable:
                case StateEnum::reset:
                    this_p->is_active = false;
                    break;
                case StateEnum::manual:
                case StateEnum::automatic:
                    this_p->is_active = true;
                }
            }
        };
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "under_carriage");

    UnderCarriage under_carriage;

    ROS_INFO("Stew: under_carriage node has started.");

    ros::spin();

    ROS_INFO("Stew: under_carriage node has terminated.");

}

