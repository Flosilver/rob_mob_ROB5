#include "ros/ros.h"
#include "prm.h"

#include <string>
#include <iostream>

PRM::PRM(int nb_nodes) : done_sub_(),
                         graph_pub_(),
                         obj_sub_(),
                         map_client_(),
                         traj_srv_(),
                         //map_srv_(),
                         map_(),
                         traj_(),
                         summits_(),
                         nb_nodes_(nb_nodes),
                         objectiv_(),
                         mapping_done_(false),
                         is_waiting_(true),
                         rate_(1.0f)
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
    graph_pub_ = n.advertise<planification::ListePoints>(graph_topic, 5);
    obj_sub_ = n.subscribe(obj_topic, 1, &PRM::objectivCallback, this);
    map_client_ = n.serviceClient<mapping::BinaryMap>(map_topic);
    traj_srv_ = n.advertiseService(traj_topic, &PRM::publishTraj, this);

    // init rate according to param
    rate_ = ros::Rate(n.param("rate", 1.0f));

    std::cout << map_.info.width << " / " << map_.info.resolution << std::endl;
}

PRM::~PRM()
{
    summits_.clear();
}

bool PRM::publishTraj(planification::Checkpoints::Request &req, planification::Checkpoints::Response &res)
{
    if (!mapping_done_ || is_waiting_)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void PRM::mappingDoneCallback(const std_msgs::Bool &msg)
{
    mapping_done_ = msg.data;
}

void PRM::objectivCallback(const geometry_msgs::PoseStamped &msg)
{
    objectiv_ = msg;
}

void PRM::generateGraph()
{
    // create every node randomly
    for (int i = 0; i < nb_nodes_; i++)
    {
        summits_.push_back(*Node::newRandNode(map_));
    }
    // check visibility and add neighbours
    for (int i = 0; i < summits_.size(); i++)
    {
        for (int j = i + 1; j < summits_.size(); j++)
        {
            if (summits_[i].canSee(summits_[j], map_))
            {
                summits_[i].addNeighbour(summits_[j]);
                summits_[j].addNeighbour(summits_[i]);
                planification::ListePoints segment;
                geometry_msgs::Point p1, p2;
                p1.x = summits_[i].x();
                p1.y = summits_[i].y();
                p2.x = summits_[j].x();
                p2.y = summits_[j].y();
                segment.points.push_back(p1);
                segment.points.push_back(p2);
                graph_pub_.publish(segment);
            }
        }
    }
}

bool PRM::waitMapping()
{
    mapping::BinaryMap map_srv;
    // std::cout << mapping_done_ << std::endl;
    if (mapping_done_)
    {
        while (!map_client_.call(map_srv))
        {
            ros::spinOnce();
        }
        map_ = map_srv.response.map;
        generateGraph();
    }
    return !mapping_done_; // if the mapping is done, we don't wait anymore
}
