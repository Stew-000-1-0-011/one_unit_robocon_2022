#pragma once

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace CRSLib
{
    namespace
    {
        template<class KeyMap, class Derived = void>
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

        protected:
            JoyToKeyButton() noexcept:
                joy_sub{ros::NodeHandle("CRSLib").subscribe<sensor_msgs::Joy>("joy", 1, &JoyToKeyButton::update, this)}
            {}
        
        public:
            template<class ... Args>
            static Derived& get_instance(Args&& ... args) noexcept
            {
                static_assert([]{return std::is_base_of_v<JoyToKeyButton, Derived>;}());

                static Derived instance(args ...);
                return instance;
            }

            bool is_being_pushed(const Buttons::Enum button) const noexcept
            {
                return latest_joy.buttons[button];
            }

            bool is_pushed_once(const Buttons::Enum button) const noexcept
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

                old_joy = latest_joy;
                latest_joy = *joy_p;
                axes = latest_joy.axes;
                on_update();
            }
        };

        template<class KeyMap>
        class JoyToKeyButton<KeyMap, void>
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

        protected:
            JoyToKeyButton() noexcept:
                joy_sub{ros::NodeHandle("CRSLib").subscribe<sensor_msgs::Joy>("joy", 1, &JoyToKeyButton::update, this)}
            {}
        
        public:
            static JoyToKeyButton& get_instance() noexcept
            {
                static JoyToKeyButton instance{};
                return instance;
            }

            bool is_being_pushed(const Buttons::Enum button) const noexcept
            {
                return latest_joy.buttons[button];
            }

            bool is_pushed_once(const Buttons::Enum button) const noexcept
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
                old_joy = latest_joy;
                latest_joy = *joy_p;
                axes = latest_joy.axes;
                on_update();
            }
        };
    }
}