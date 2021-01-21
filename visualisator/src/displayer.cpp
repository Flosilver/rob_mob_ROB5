#include "displayer.h"

#include <string>
#include <iostream>

Displayer::Displayer() : map_client_(),
                         traj_client_(),
                         odom_sub_(),
                         tree_sub_(),
                         gridmap_(),
                         display_map_(),
                         map_srv_(),
                         traj_(),
                         mapping_done_(false),
                         final_map_get_(false),
                         rate_(1.0f)
{
    ros::NodeHandle n("~");

    // Get topics and services
    std::string map_srv(n.param<std::string>("bin_map_srv", "/binary_map"));
    //std::string traj_srv(n.param<std::string>("traj_srv", "/checkpoints"));
    std::string odom_topic(n.param<std::string>("odom_topic", "/odom"));
    //std::string rrt_topic(n.param<std::string>("rrt_topic", "/segments_rrt"));
    std::string map_done_topic(n.param<std::string>("map_done_topic", "/mapping_done"));

    // Set service & client, publisher & subscriber
    map_client_ = n.serviceClient<mapping::BinaryMap>(map_srv);
    // traj_client_ = n.serviceClient<control::Traj>(traj_srv);
    odom_sub_ = n.subscribe(odom_topic, 1, &Displayer::odomCallback, this);
    // tree_sub_ = n.subscribe(rrt_topic, 1, &Displayer::treeCallback, this);
    map_done_sub_ = n.subscribe(map_done_topic, 1, &Displayer::mapDoneCallback, this);

    // Init robot_
    robot_.pos = cv::Point();
    robot_.radius = n.param("robot_size", 10);
    robot_.color = cv::Vec3b(200, 0, 0);

    // Init tree_
    tree_.thickness = n.param("tree_thickness", 1);
    tree_.color = cv::Vec3b(100, 100, 100);

    // init rate according to param
    rate_ = ros::Rate(n.param("rate", 1.0f));

    // open OpenCV window
    cv::namedWindow(WINDOW_NAME);
}

Displayer::~Displayer()
{
    tree_.nodes.clear();
}

void Displayer::odomCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Pose robot_pos = msg.pose.pose;
    unsigned int mx, my;
    if (gridmap_.inMapBounds(robot_pos.position.x, robot_pos.position.y))
    {
        gridmap_.worldToMap(robot_pos.position.x, robot_pos.position.y, mx, my);
        robot_.pos.x = my;
        robot_.pos.y = mx;
    }
}

// void Displayer::treeCallback(const control::ListePoints &msg)
// {
//     unsigned int mx, my;
//     for (const geometry_msgs::Point &pt : msg.points)
//     {
//         tree.nodes.push_back(cv::Point(pt.x, pt.y));
//         // if (gridmap.inMapBounds(pt.x, pt.y))
//         // {
//         //     gridmap.worldToMap(pt.x, pt.y, mx, my);
//         //     tree.nodes.push_back(cv::Point(my, mx));

//         // }
//         // else
//         // {
//         //     std::cout << "***WARNING:\n" << pt << " not in bound!\n";
//         // }
//     }
// }

void Displayer::mapDoneCallback(const std_msgs::Bool &msg)
{
    mapping_done_ = msg.data;
    if (mapping_done_ && !final_map_get_)
    {
        while (!map_client_.call(map_srv_))
            ;
        gridmap_.setMap(map_srv_.response.map);
        final_map_get_ = true;
    }
}

bool Displayer::mappingDone()
{
    if (!mapping_done_ && map_client_.call(map_srv_))
    {
        gridmap_.setMap(map_srv_.response.map);
        cv::cvtColor(gridmap_.binaryMap(), display_map_, cv::COLOR_GRAY2RGB);

        // Affichage du robot dans la map
        if (robot_.pos.x != 0 && robot_.pos.y != 0)
        {
            cv::circle(display_map_, robot_.pos, robot_.radius, robot_.color, CV_FILLED);
        }

        // if (!display_map_.empty())
        // {
        cv::imshow(WINDOW_NAME, display_map_);
        // }
        cvWaitKey(1);
    }
    rate_.sleep();
    return mapping_done_;
}

void Displayer::display()
{
    // if (map_client_.call(map_srv_))
    // {
    //     gridmap_.setMap(map_srv_.response.map);
    //     cv::cvtColor(gridmap_.binaryMap(), display_map_, cv::COLOR_GRAY2RGB);
    // }

    if (!gridmap_.binaryMap().empty())
    {
        display_map_ = cv::Mat::zeros(display_map_.rows, display_map_.cols, CV_8U);
        cv::cvtColor(gridmap_.binaryMap(), display_map_, cv::COLOR_GRAY2RGB);

        // Affichage du robot dans la map
        if (robot_.pos.x != 0 && robot_.pos.y != 0)
        {
            cv::circle(display_map_, robot_.pos, robot_.radius, robot_.color, CV_FILLED);
        }

        cv::imshow(WINDOW_NAME, display_map_);
        cvWaitKey(1);
    }
    rate_.sleep();
}

void Displayer::displayTree(cv::Mat &dest)
{
    for (size_t i = 1; i < tree_.nodes.size(); i += 2)
    {
        cv::line(dest, tree_.nodes.at(i - 1), tree_.nodes.at(i), tree_.color, tree_.thickness);
    }
}