#include "ros/ros.h"
#include "supervisor.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <memory>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "supervisor");
    ros::NodeHandle n;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf2(buffer);
    std::shared_ptr<Supervisor> supervisor(new Supervisor(&buffer));

    ros::Time time = ros::Time::now();
    ros::Duration delay(3.);

    supervisor->init();

    while (ros::ok())
    {
        if (ros::Time::now() - time > delay)
        {
            supervisor->spin();
            time = ros::Time::now();
        }

        supervisor->show();
        ros::spinOnce();
    }
    return 0;
}