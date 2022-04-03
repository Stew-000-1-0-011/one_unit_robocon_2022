#include <cstdint>

#include <ros/ros.h>
#include <one_unit_robocon_2022/State.h>

#include <one_unit_robocon_2022/state.hpp>

namespace CRSLib
{
    namespace
    {
        namespace StateManagerImplement
        {
            auto empty_lambda = [](const OneUnitRobocon2022::StateEnum) noexcept {};
        }
        // ここら辺よくわからない...型どこで変わるんだろ。とりあえずデフォルト引数にラムダ式直入れはやめよう。
        // template<typename StateEnum, auto user_callback = [](const OneUnitRobocon2022::StateEnum) noexcept {}>
        template<typename StateEnum, auto user_callback = StateManagerImplement::empty_lambda>
#ifdef __cpp_concepts
        requires requires(StateEnum state)
        {
            std::is_same_v<std::underlying_type_t<StateEnum>, std::uint8_t>;
            {user_callback(state)} noexcept -> std::same_as<void>;
        }
#endif
        class StateManager final
        {
            ros::Publisher pub{};

            StateEnum state{StateEnum::disable};
            ros::Subscriber sub;

        private:
            StateManager(ros::NodeHandle& nh) noexcept:
                pub{nh.advertise<one_unit_robocon_2022::State>("stew_state", 10)},
                sub{nh.subscribe<one_unit_robocon_2022::State>("stew_state", 10, &StateManager::callback, this)}
            {}
        
        public:
            static StateManager& get_instance(ros::NodeHandle& nh) noexcept
            {
                static StateManager instance{nh};
                return instance;
            }

            StateEnum get_state() noexcept
            {
                return state;
            }

            void set_state(const StateEnum state) noexcept
            {
                this->state = state;
                one_unit_robocon_2022::State state_msg;
                state_msg.data = static_cast<std::uint8_t>(state);
                pub.publish(state_msg);
            }

        private:
            void callback(const one_unit_robocon_2022::State::ConstPtr& msg_p) noexcept
            {
                state = static_cast<StateEnum>(msg_p->data);
                user_callback(state);
            }
        };
    }
}