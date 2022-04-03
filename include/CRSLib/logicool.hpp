/*
XInputモードにしか対応していない。
DirectInputモードやその切り替えを十分に調べたら実装するかも。
まあ、コントローラーが保持する状態はx/Dモードのどちらかなのか以外にはない(はず)なので、間違えて切り替えてもきっと大丈夫。
*/


#pragma once

#include <cstdint>

#include <sensor_msgs/Joy.h>

#include "joy_to_key_button.hpp"

namespace CRSLib
{
    namespace
    {
        namespace LogicoolDetail
        {
            // なぜかnested classにするとstatic constexpr メンバ変数にできない。やむなく名前空間で隠した。
            struct Trigger final
            {
                std::uint8_t value{};

                constexpr Trigger(const std::uint8_t value) noexcept:
                    value{value}
                {}

                constexpr operator std::uint8_t() const noexcept
                {
                    return value;
                }

                constexpr Trigger& operator=(const std::uint8_t value) noexcept
                {
                    this->value = value;
                    return *this;
                }
            };
        }

        class Logicool;

        struct LogicoolXInputKeyMap
        {
            friend class Logicool;

            struct Axes final
            {
                enum Enum : std::uint8_t
                {
                    l_stick_LR = 0,
                    l_stick_UD,
                    l_trigger_dummy,
                    r_stick_LR,
                    r_stick_UD,
                    r_trigger_dummy,
                    cross_LR,
                    cross_UD,

                    N
                };
            
                static constexpr LogicoolDetail::Trigger l_trigger{l_trigger_dummy};
                static constexpr LogicoolDetail::Trigger r_trigger{r_trigger_dummy};
            };

            struct Buttons final
            {
                enum Enum : std::uint8_t
                {
                    a = 0,
                    b,
                    x,
                    y,
                    lb,
                    rb,
                    back,
                    start,
                    l_push,
                    r_push,

                    N
                };
            };
        };

        // // Logicoolにはメンバ変数がないので継承先のデストラクタをオーバーライドする必要はない。
        // JoyパッケージのLRトリガーの仕様がひどい。
        // (rostopic echo /joy で見てみるとわかるけど、一度押す前はニュートラルで0.0、それ以降は-1.0となる。)
        // これに対応するために、しぶしぶ「起動後一回以上Lトリガーが押されたか」「起動後一回以上Rトリガーが押されたか」を保持する変数を
        // Logicool用のクラスに用意する。
        // 伴って、Joyの起動のたびにこいつを初期化せねばならない。

        class Logicool final : public JoyToKeyButton<LogicoolXInputKeyMap>
        {
            std::int8_t l_trigger_neutral{};
            std::int8_t r_trigger_neutral{};

        public:
            Logicool(ros::NodeHandle& nh) noexcept:
                JoyToKeyButton{nh}
            {}

            bool is_being_pushed(const LogicoolDetail::Trigger trigger) const noexcept
            {
                // triggerクラスは隠蔽されてるので、渡される引数はl_triggerかr_triggerに必ず等しい。
                switch(trigger)
                {
                case Axes::l_trigger:
                    return axes[Axes::l_trigger] != l_trigger_neutral;

                case Axes::r_trigger:
                    return axes[Axes::r_trigger] != r_trigger_neutral;
                
                default:
                    ROS_ERROR("Stew: invalid argument was passed to CRSLib::Logicool::is_being_pushed. You must not use LogicoolDetail::Trigger directly.");
                    return false;
                }
            }

            bool is_pushed_once(const LogicoolDetail::Trigger trigger) const noexcept
            {
                switch(trigger)
                {
                case Axes::l_trigger:
                    return axes[Axes::l_trigger] != l_trigger_neutral && old_joy.axes[Axes::l_trigger] == l_trigger_neutral;
                
                case Axes::r_trigger:
                    return axes[Axes::r_trigger] != r_trigger_neutral && old_joy.axes[Axes::r_trigger] == r_trigger_neutral;
                
                default:
                    ROS_ERROR("Stew: invalid argument was passed to CRSLib::Logicool::is_pushed_once. You must not use LogicoolDetail::Trigger directly.");
                    return false;
                }
            }

            void init() noexcept
            {
                l_trigger_neutral = 0;
                r_trigger_neutral = 0;
            }

        private:
            virtual void on_update() noexcept final
            {
                if(l_trigger_neutral != -1 && is_being_pushed(Axes::l_trigger)) l_trigger_neutral = -1;
                if(r_trigger_neutral != -1 && is_being_pushed(Axes::r_trigger)) r_trigger_neutral = -1;
            }
        };
    }
}