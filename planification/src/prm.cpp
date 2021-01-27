#include "ros/ros.h"
#include "prm.h"

#include <string>

PRM::PRM(int nb_nodes) : done_sub_(),
graph_pub_(),
obj_sub_(),
map_client_(),
traj_client_(),
map_srv_(),
map_(),
traj_(),
summits_(),
nb_nodes_(nb_nodes),
objectiv_(),
mapping_done(false),
is_waiting_(true)
{
    ros::NodeHandle n("~");

    // Get topics and services
    std::string done_topic(n.param<std::string>("done_topic", "/mapping_done"));
    std::string graph_topic(n.param<std::string>("graph_topic", "/segments_rrt"));
    std::string obj_topic(n.param<std::string>("objectiv_topic", "/move_base_simple/goal"));

    std::string map_topic(n.param<std::string>("map_srv", "/binary_map"));
    std::string traj_topic(n.param<std::string>("traj_srv", "/checkpoints"));

    // Set service & client, publisher & subscriber
    done_sub_ = n.subscribe(done_topic, 1, &PRM::mappingDoneCallback, this);
    graph_pub_ = n.advertise<planification::ListePoints>(tree_topic, 5);
    obj_sub_ = n.subscribe(obj_topic, 1, &PRM::objectivCallback, this);
    map_client_ = n.serviceClient<mapping::BinaryMap>(map_topic);
    traj_srv_ = n.advertiseService(traj_topic, &PRM::publish_traj, this);
}

PRM::~PRM(){
    summits_.clear();
}