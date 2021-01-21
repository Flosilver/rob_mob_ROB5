#include "ros/ros.h"
#include "explorer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_explo");
    ros::NodeHandle n;
    Explorer xplo;
    ros::spin();

    return 0;
}