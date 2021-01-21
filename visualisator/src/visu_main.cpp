#include "ros/ros.h"
#include "displayer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualisator");
    ros::NodeHandle n;
    Displayer d;

    while (ros::ok())
    {
        // while (!d.mappingDone())
        // {
        //     ros::spinOnce();
        // }
        if (d.mappingDone()) { d.display(); }
        ros::spinOnce();
    }
    return 0;
}