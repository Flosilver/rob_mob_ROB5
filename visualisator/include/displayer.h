#ifndef __DISPLAYER_H__
#define __DISPLAYER_H__

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"

#include <vector>
#include <opencv2/highgui.hpp>

#include "gridmap2d.h"
#include "trajectory.h"
#include <mapping/BinaryMap.h>
// #include "control/ListePoints.h"

#define WINDOW_NAME "Display Window"

class Displayer
{
public:
    struct Robot
    {
        cv::Point pos;
        int radius;
        cv::Vec3b color;
    };
    struct Tree
    {
        std::vector<cv::Point> nodes;
        unsigned int thickness;
        cv::Vec3b color;
    };

private:
    ros::ServiceClient map_client_;
    ros::ServiceClient traj_client_;
    ros::Subscriber odom_sub_;
    ros::Subscriber tree_sub_;
    ros::Subscriber map_done_sub_;

    gridmap_2d::GridMap2D gridmap_;
    cv::Mat display_map_;
    mapping::BinaryMap map_srv_;
    // control::Traj traj_srv_;
    Trajectory traj_;
    Robot robot_;
    Tree tree_;
    bool mapping_done_;
    bool final_map_get_;

    ros::Rate rate_;
    ros::Time traj_time_;
    ros::Duration traj_update_time_;

public:
    Displayer();
    ~Displayer();

    // Topics callback functions

    void odomCallback(const nav_msgs::Odometry &msg);
    // void treeCallback(const control::ListePoints &msg);
    void mapDoneCallback(const std_msgs::Bool &msg);

    bool mappingDone();
    void display();
    void displayTree(cv::Mat &dest);
};

#endif // __DISPLAYER_H__