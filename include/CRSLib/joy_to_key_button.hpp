#pragma once

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace CRSLib
{
    namespace
    {
        template<class KeyMap>
        class JoyToKeyButton
        {
        public:
            using Axes = KeyMap::Axes;
            using Buttons = KeyMap::Buttons;
            std::vector<float> axes = std::vector<float>(Axes::N);

        protected:
            sensor_msgs::Joy latest_joy{[]{sensor_msgs::Joy msg; msg.axes = std::vector<float>(Axes::N, 0); msg.buttons = std::vector<std::int32_t>(Buttons::N, 0); return msg;}()};
            sensor_msgs::Joy old_joy{[]{sensor_msgs::Joy msg; msg.axes = std::vector<float>(Axes::N, 0); msg.buttons = std::vector<std::int32_t>(Buttons::N, 0); return msg;}()};

        private:
            ros::Subscriber joy_sub{};

        public:
            JoyToKeyButton(ros::NodeHandle& nh) noexcept:
                joy_sub{nh.subscribe<sensor_msgs::Joy>("joy", 1, &JoyToKeyButton::update, this)}
            {}

            bool is_being_pushed(const Buttons::Enum button) const noexcept
            {
                return latest_joy.buttons[button];
            }

            bool is_pushed_once(const Buttons::Enum button) noexcept
            {
                return old_joy.buttons[button] && !latest_joy.buttons[button];
            }

        protected:
            virtual void on_update() noexcept
            {}

        private:

            void update(const sensor_msgs::Joy::ConstPtr& joy_p) noexcept
            {
                if(joy_p->axes.size() != Axes::N)
                {
                    ROS_WARN("size of axes is differ from Axes::N.");
                }
                else if(joy_p->buttons.size() != Buttons::N)
                {
                    ROS_WARN("size of buttons is differ from Buttons::N.");
                }
                else
                {
                    old_joy = latest_joy;
                    latest_joy = *joy_p;
                    axes = latest_joy.axes;
                    on_update();
                }
            }
        };
    }
}