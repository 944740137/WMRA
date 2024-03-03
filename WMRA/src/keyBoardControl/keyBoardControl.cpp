#include "ros/ros.h"
#include "Mobile_Platform/MobileBase.h"
#include "Mobile_Platform/kvaser.h"
int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"start");
    ROBOT_MOBILE_BASE robot_base(0);
    ros::Rate r(1);

    while (ros::ok())
    {
        robot_base.run();
        ros::spinOnce();
    }
    
    return 0;
}
