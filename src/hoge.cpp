#include "CRSLib/shirasu.hpp"

using namespace CRSLib;

int main()
{
    ros::NodeHandle nh;
    CanPublisher can_pub{nh};
    can_pub.can_publish<int>(1, 1);
}

#include <algorithm>