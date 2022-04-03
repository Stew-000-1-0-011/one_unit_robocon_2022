#include <cstdint>

#include <ros/ros.h>
#include <one_unit_robocon_2022/State.h>

namespace CRSLib
{
    namespace
    {
        template<typename State, auto user_callback = [](const State) noexcept {}>
#ifdef __cpp_concepts
        requires requires(State state)
        {
            std::is_same_v<std::underlying_type_t<State>, std::uint8_t>;
            {user_callback(state)} noexcept -> std::same_as<void>;
        }
#endif
        class StateManager final
        {
            ros::Publisher pub{};

            State state{State::disable};
            ros::Subscriber sub;

        public:
            StateManager(ros::NodeHandle& nh) noexcept:
                pub{nh.advertise<one_unit_robocon_2022::State>("stew_state", 10)},
                sub{nh.subscribe<one_unit_robocon_2022::State>("stew_state", 10, &StateManager::callback, this)}
            {}

            State get_state() noexcept
            {
                return state;
            }

            void set_state(const State state) noexcept
            {
                this->state = state;
                one_unit_robocon_2022::State state_msg;
                state_msg.data = static_cast<std::uint8_t>(state);
                pub.publish(state_msg);
            }

        private:
            void callback(const one_unit_robocon_2022::State::ConstPtr& msg_p) noexcept
            {
                state = static_cast<State>(msg_p->data);
                user_callback(state);
            }
        };
    }
}