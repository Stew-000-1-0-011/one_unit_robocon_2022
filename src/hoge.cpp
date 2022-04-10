#include <ros/ros.h>

// #include "CRSLib/can_subscriber.hpp"
#include "CRSLib/rosparam_util.hpp"
using namespace CRSLib;
using namespace CRSLib::RosparamUtil;
int main()
{
    XmlRpc::XmlRpcValue list;
    ros::NodeHandle nh;
    nh.getParam("list", list);
    double freq = read_param<double>(list, "freq");
    is_positive.template operator()<double>(freq);
}