#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <random>

namespace OneUnitRobocon2022
{
    namespace
    {
        class MaybeUBNode final : public nodelet::Nodelet
        {
            std::random_device seed_gen{};
            std::mt19937_64 engine{seed_gen()};

            int var{};
            ros::Timer tim;
            ros::Timer tim2;
            ros::Publisher pub;
            ros::Subscriber sub;

            virtual void onInit() override
            {
                ros::NodeHandle nh = getMTNodeHandle();
                pub = nh.advertise<std_msgs::Empty>("Event", 1);
                tim = nh.createTimer(ros::Duration(1/ 10000), &MaybeUBNode::callback, this);
                tim2 = nh.createTimer(ros::Duration(1/ 10000), &MaybeUBNode::callback, this);
                // sub = nh.subscribe<std_msgs::Empty>("Event", 100, &MaybeUBNode::callback, this);
            }

            void pub_event(const ros::TimerEvent&)
            {
                pub.publish(std_msgs::Empty());
            }

            // void callback(const std_msgs::EmptyConstPtr&)
            void callback(const ros::TimerEvent&)
            {
                var = 0;

                ros::Duration(engine() % 2).sleep();

                if(var) ROS_INFO("UB!!!\n");
                else ROS_INFO("SAFE?\n");

                var = 1;

                ros::Duration(engine() % 2).sleep();

                // ros::Duration(1).sleep();
                // ROS_INFO("CALLBACK.\n");
            }
        };
    }
}

PLUGINLIB_EXPORT_CLASS(OneUnitRobocon2022::MaybeUBNode, nodelet::Nodelet)
