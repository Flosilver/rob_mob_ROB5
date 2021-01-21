#include "mapper.h"

#include <string>

Mapper::Mapper() : map_client_(),
                   map_serv_(),
                   done_pub_(),
                   gridmap_(),
                   map_srv_(),
                   done_(false),
                   rate_(1.0f)
{
    ros::NodeHandle n("~");

    // Get params
    rate_ = ros::Rate(n.param("rate", 1.0f));

    // Get topics and services
    //std::string map_srv(n.param<std::string>("map_srv", "/static_map"));
    std::string binary_map_srv(n.param<std::string>("bin_map_srv", "/binary_map"));
    std::string done_topic(n.param<std::string>("done_topic", "/mapping_done"));

    // Initialize service server, client and publisher
    // map_client_ = n.serviceClient<nav_msgs::GetMap>(map_srv);
    map_serv_ = n.advertiseService(binary_map_srv, &Mapper::publish, this);
    done_pub_ = n.advertise<std_msgs::Bool>(done_topic, 1, true);
}

Mapper::~Mapper() {}

bool Mapper::publish(mapping::BinaryMap::Request &req, mapping::BinaryMap::Response &res)
{
    if (!gridmap_.binaryMap().empty())
    {
        res.map = gridmap_.toOccupancyGridMsg();
        return true;
    }
    else
        return false;
}