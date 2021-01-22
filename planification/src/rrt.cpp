#include "rrt.h"
#include <string>

RRT::RRT(float expand_dist = 1.0f) : done_sub_(),
tree_sub_(),
obj_sub_(),
map_client_(),
traj_srv_(),
map_(),
traj_(),
ts(expand_dist),
tg(expand_dist),
objectiv_(),
is_waiting_(true),
done_(false)
{
    ros::NodeHandle n("~");

    // Get topics and services
    std::string done_topic(n.param<std::string>("map_done_topic", "/mapping_done"));
    std::string tree_topic(n.param<std::string>("tree_topic", "/segments_rrt"));
    std::string obj_topic(n.param<std::string>("obj_topic", "/move_base_simple/goal"))
    std::string map_srv(n.param<std::string>("map_srv", "/binary_map"));
    std::string traj_srv(n.param<std::string>("traj_srv", "/checkpoints"));

    // Set service & client, publisher & subscriber
    done_sub_ = n.subscribe(done_topic, 1, &RRT::doneCallback, this);
    tree_pub_ = n.advertise<planification::ListePoints>(tree_topic, 5);
    obj_sub_ = n.subscribe(obj_topic, 1, &RRT::objectivCallback, this);
    map_client_ = n.serviceClient<mapping::BinaryMap>(map_srv);
    traj_srv_ = n.advertiseService(traj_srv, &RRT::publish_traj, this);
}

RRT::~RRT() {}

bool RRT::publish_traj(planification::Checkpoints::Request& req, planification::Checkpoints::Response& res) 
{
    // If the node is still waiting to start or if the traj is not ready yet
    if (is_waiting_ || !done_) { return false;}

    return true;
}
