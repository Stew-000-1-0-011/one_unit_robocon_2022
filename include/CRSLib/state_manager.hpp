#include <cstdint>

#include <ros/ros.h>
#include <one_unit_robocon_2022/State.h>

#ifndef __cpp_concepts
#include "StewLib/has_member.hpp"
#endif

#include "one_unit_robocon_2022/state.hpp"

#ifndef __cpp_concepts
Stew_define_has_member(callback, callback)
#endif

namespace CRSLib
{
    namespace
    {
        template<class T>
        struct StateManagerCallback;

        template<typename StateEnum, class T>
#ifdef __cpp_concepts
        requires requires(StateEnum state)
        {
            // Message型がuint8固定なので
            std::is_same_v<std::underlying_type_t<StateEnum>, std::uint8_t>;
        }
#endif
        class StateManager final
        {
            inline static ros::Publisher pub{};
            inline static ros::Subscriber sub{};

            StateEnum state{StateEnum::disable};
            T * this_p;

        public:
            StateManager(ros::NodeHandle& nh, T *const this_p) noexcept:
                this_p{this_p}
            {
                if(!pub) pub = nh.advertise<one_unit_robocon_2022::State>("stew_state", 10);
                if(!sub) sub = nh.subscribe<one_unit_robocon_2022::State>("stew_state", 10, &StateManager::callback, this);
            }

            StateEnum get_state() const noexcept
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

#ifdef __cpp_concepts
                if constexpr(requires{StateManagerCallback<T>::callback(this_p, state);})
                {
                    StateManagerCallback<T>::callback(this_p, state);
                }
#else
                if constexpr(StewLib::HasMember::has_callback_v<StateManagerCallback<T>>)
                {
                    StateManagerCallback<T>::callback(this_p, state);
                }
#endif
            }
        };
    }
}