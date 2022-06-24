// #include "one_unit_robocon_2022/manual_commander.hpp"

// using namespace CRSLib::RosparamUtil::Implement;

// static_assert(is_string_like<const int>);

// #include "CRSLib/can_subscriber_manager.hpp"

// using namespace CRSLib;

// struct Hoge{};

// struct HogeCallback final
// {
//     void callback(int) noexcept
//     {}
// };

// struct HogeBuffer final
// {
//     bool push(const can_plugins::Frame::ConstPtr&) noexcept
//     {
//         return true;
//     }

//     int get_value() const noexcept
//     {
//         return 0;
//     }
// };

// int main()
// {
//     ros::NodeHandle nh{};
//     CRSLib::CanSubscriberManager can_subscriber_manager{nh, CanSubscriber<Hoge, HogeCallback, HogeBuffer>{0x04}};
// }

// #include <ros/ros.h>
// #include <pluginlib/class_list_macros.h>
// #include <nodelet/nodelet.h>

// struct MaybeUBNode final : public nodelet::Nodelet
// {
//     int var{};
//     ros::Timer timer1;
//     ros::Timer timer2;

//     virtual void onInit() override
//     {
//         ros::NodeHandle nh = getMTNodeHandle();
//         timer1 = nh.createTimer(ros::Duration(1.0), &MaybeUBNode::callback, this);
//         timer2 = nh.createTimer(ros::Duration(1.0), &MaybeUBNode::callback, this);
//     }

//     void callback(const ros::TimerEvent&)
//     {
//         var = 0;
//     }
// };

// PLUGINLIB_EXPORT_CLASS(MaybeUBNode, nodelet::Nodelet)

// #include <atomic>
// #include <iostream>

int main()
{
    
}