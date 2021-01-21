#include "loader.h"

Loader::Loader() : Mapper()
{
    ros::NodeHandle n("~");

    // Get topic and service names
    std::string map_srv(n.param<std::string>("map_srv", "/static_map"));

    // Initialize subscriber and publisher
    map_client_ = n.serviceClient<nav_msgs::GetMap>(map_srv);
}

Loader::~Loader() {}

void Loader::spin()
{
    std_msgs::Bool done;
    if (!done_ && map_client_.call(map_srv_))
    {
        done_ = true;
        gridmap_.setMap(map_srv_.response.map, false, 60);
        gridmap_.inflateMap(0.7);
        done.data = done_;
        done_pub_.publish(done);
    }
    rate_.sleep();
}