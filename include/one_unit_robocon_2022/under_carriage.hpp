/*
base_controllerを参考にした。
機体に固定された座標での平面上移動におけるTwistを受け取り、各モーターへの速度をslcan_bridgeに向けてpublishしている。

どうせならURDFあたりから勝手にタイヤの位置情報とか読み出せるようにしたい。
*/

// 標準ライブラリ
#ifdef __cpp_lib_math_constants
#include <numbers>
#endif

#include <atomic>

// ROSライブラリ((ほぼ)自動生成のものを含む)
#include <ros/ros.h>
#include "one_unit_robocon_2022/Twist.h"

// StewLibライブラリ(拙作で本当につたないのでダメ出し待ってます)(C++20以上に対応(concepts無くてもギリ動くぐらい))
#include "StewLib/Math/vec2d.hpp"

// crs_libライブラリ(同上。)
#include "crs_lib/shirasu.hpp"
#include "crs_lib/state_manager.hpp"
#include "crs_lib/rosparam_util.hpp"

// 今回だけ使うヘッダ
#include "state.hpp"

namespace OneUnitRobocon2022
{
    namespace
    {
        struct Wheel final
        {
            uint32_t base_id{};
            StewLib::Math::Vec2D<double> position{};
            StewLib::Math::Vec2D<double> direction{};

            Wheel() = default;
            Wheel& operator=(Wheel&&) = default;

            Wheel(CRSLib::RosparamUtil::StewXmlRpc& wheel_list) noexcept
            {
                using namespace CRSLib::RosparamUtil;

                base_id = read_param<int>(wheel_list, "base_id");

                auto position_list = get_param(wheel_list, "position");
                position = {read_param<double>(position_list, 0), read_param<double>(position_list, 1)};
                assert_param(position, norm2(position), StewLib::Math::Vec2D<double>{}, "position");

                auto direction_list = get_param(wheel_list, "direction");
                direction = {read_param<double>(direction_list, 0), read_param<double>(direction_list, 1)};
                assert_param(direction, norm2(direction), StewLib::Math::Vec2D<double>{}, "direction");
            }
        };

        class UnderCarriage final
        {
            friend struct ::CRSLib::StateManagerCallback<UnderCarriage>;

            // rosparamを使って初期化したいconstな非静的メンバ変数をまとめて初期化するためにクラスで包んでいる
            const struct RosParamData final
            {
                double control_freq{};
                double max_wheel_vel{};
                double max_wheel_acc{};
                double wheel_radius{};
                bool is_send_target_velocity{};

                /// TODO: N個のタイヤに対応
                // std::vector<Wheel> wheels = std::vector<wheel>(n);
                Wheel wheels[4]{};

                RosParamData(ros::NodeHandle& nh) noexcept
                {
                    // using namespace CRSLib::RosparamUtil;と書いたときとほぼ同じ。
                    using CRSLib::RosparamUtil::assert_param;
                    using CRSLib::RosparamUtil::get_param;
                    using CRSLib::RosparamUtil::xml_rpc_cast;
                    using CRSLib::RosparamUtil::read_param;
                    using CRSLib::RosparamUtil::StewXmlRpc;

                    std::optional<StewXmlRpc> under_carriage_opt = get_param(nh, "under_carriage");

                    control_freq = read_param<double>(under_carriage_opt, "control_freq");
                    assert_param(control_freq, control_freq > 0, 1000, "control_freq");

                    max_wheel_vel = read_param<double>(under_carriage_opt, "max_wheel_vel");
                    assert_param(max_wheel_vel, max_wheel_vel >= 0, 0, "max_wheel_vel");

                    max_wheel_acc = read_param<double>(under_carriage_opt, "max_wheel_acc");
                    assert_param(max_wheel_acc, max_wheel_acc >= 0, 0, "max_wheel_acc");

                    wheel_radius = read_param<double>(under_carriage_opt, "wheel_radius");
                    assert_param(wheel_radius, wheel_radius > 0, 0, "wheel_radius");

                    auto wheels_opt = get_param(under_carriage_opt, "wheels");
                    for(int i = 0; i < 4; ++i) if(auto wheel_opt = get_param(wheels_opt, i); wheel_opt.has_value())
                    {
                        wheels[i] = *wheel_opt;
                    }

                    is_send_target_velocity = read_param<bool>(under_carriage_opt, "is_send_target_velocity");
                }
            } ros_param_data;

            const struct CalcConstant final
            {
                StewLib::Math::Vec2D<double> linear_factor[4];
                double angular_factor[4];
                double distance[4];

                CalcConstant(const Wheel(& wheels)[4]) noexcept
                {
                    using namespace StewLib::Math;

                    for(int i = 0; i < 4; ++i)
                    {
                        linear_factor[i] = wheels[i].direction / norm(wheels[i].direction);
#ifdef __cpp_lib_math_constants
                        angular_factor[i] = rot(wheels[i].position, std::numbers::pi) * linear_factor[i];
#else
                        angular_factor[i] = rot(wheels[i].position, 3.141592653589793116) * linear_factor[i];
#endif
                        distance[i] = norm(wheels[i].position);
                    }
                }
            } constant{ros_param_data.wheels};

            static_assert(std::atomic<double>::is_always_lock_free);
            static_assert(std::atomic<bool>::is_always_lock_free);
            StewLib::Math::Vec2D<std::atomic<double>> body_linear_vel{};
            std::atomic<double> body_angular_vel{};
            std::atomic<double> wheels_vel[4]{};
            std::atomic<bool> is_active{false};

            CRSLib::Shirasu motors[4]{};

            ros::Timer publish_timer;
            ros::Subscriber body_twist_sub;
            CRSLib::StateManager<StateEnum, UnderCarriage> state_manager;

        public:
            UnderCarriage(ros::NodeHandle& nh) noexcept:
                ros_param_data{nh},
                publish_timer{nh.createTimer(ros::Duration(1 / ros_param_data.control_freq), &UnderCarriage::publish_timer_callback, this)},
                body_twist_sub{nh.subscribe<one_unit_robocon_2022::Twist>("body_twist", 1, &UnderCarriage::body_twist_callback, this)},
                state_manager{nh, this}
            {
                using namespace CRSLib;

                for(int i = 0; i < 4; ++i)
                {
                    motors[i] = Shirasu(nh, ros_param_data.wheels[i].base_id);
                }
            }

        private:
            void publish_timer_callback(const ros::TimerEvent&) noexcept
            {
                if(!is_active) return;

                calc_wheels_vela();

                for(int i = 0; i < 4; ++i)
                {
                    ROS_INFO("Stew: DEBUG: wheels_vel: %lf", static_cast<double>(wheels_vel[i]));
                    motors[i].send_target(wheels_vel[i]);
                    if(ros_param_data.is_send_target_velocity) motors[i].send_debug_target(wheels_vel[i]);
                }
            }

            void body_twist_callback(const one_unit_robocon_2022::Twist::ConstPtr& msg_p) noexcept
            {
            
                body_linear_vel.x = msg_p->linear_x;
                body_linear_vel.y = msg_p->linear_y;
                body_angular_vel = msg_p->angular_z;
            }
            
            /// TODO: 速さ上限の設定と加速度の大きさ上限の設定
            inline void calc_wheels_vela() noexcept
            {
                for(int i = 0; i < 4; ++i)
                {
                    wheels_vel[i] = (constant.linear_factor[i] * body_linear_vel + constant.angular_factor[i] * body_angular_vel) / constant.distance[i];
                }
            }
        };
    }
}

namespace CRSLib
{
    template<>
    struct StateManagerCallback<OneUnitRobocon2022::UnderCarriage> final
    {
        static void callback(OneUnitRobocon2022::UnderCarriage *const this_p, const OneUnitRobocon2022::StateEnum state) noexcept
        {
            using namespace OneUnitRobocon2022;
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
