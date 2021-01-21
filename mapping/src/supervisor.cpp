#include "supervisor.h"
#include <iostream>

Supervisor::Supervisor(tf2_ros::Buffer *buffer) : Mapper(),
                                                  check_(),
                                                  robot_frame_(),
                                                  map_frame_(),
                                                  tf_(buffer),
                                                  robot_pos_init_()
{
    ros::NodeHandle n("~");

    // Get topic and service names
    std::string map_srv(n.param<std::string>("map_srv", "/dynamic_map"));

    // Initialize subscriber and publisher
    map_client_ = n.serviceClient<nav_msgs::GetMap>(map_srv);

    robot_frame_ = n.param<std::string>("robot_frame", "base_footprint");

    // cv::namedWindow("check");
}

Supervisor::~Supervisor()
{
}

void Supervisor::init()
{
    while (!map_client_.call(map_srv_))
        ;
    nav_msgs::OccupancyGrid grid(map_srv_.response.map);
    gridmap_.setMap(grid, false, 60);
    map_frame_ = grid.header.frame_id;

    // Wait for the transform to be available
    while (!tf_->canTransform(robot_frame_, map_frame_, ros::Time(0.0), ros::Duration(1.0)))
        ;
    robot_pos_init_ = robotPos();
}

cv::Point Supervisor::robotPos()
{
    geometry_msgs::PointStamped origin_in_robot;
    origin_in_robot.header.frame_id = robot_frame_;
    geometry_msgs::PointStamped robot_pos_in_map;
    tf_->transform(origin_in_robot, robot_pos_in_map, map_frame_);

    unsigned int mx, my;
    gridmap_.worldToMap(robot_pos_in_map.point.x, robot_pos_in_map.point.y, mx, my);
    cv::Point robot_pos(my, mx);
    return robot_pos;
}

void Supervisor::spin()
{
    std_msgs::Bool done;
    if (!done_ && map_client_.call(map_srv_))
    {
        nav_msgs::OccupancyGrid grid(map_srv_.response.map);
        gridmap_.setMap(grid, false, 60);
        gridmap_.inflateMap(0.7);
        check_ = gridmap_.binaryMap().clone();
        map_frame_ = grid.header.frame_id;

        cv::circle(check_, robot_pos_init_, 70, cv::Vec3b(0, 0, 0), CV_FILLED);
        
        // end condition
        cv::floodFill(check_, robotPos(), cv::Scalar(155));
        if ((int)check_.at<uchar>(0, 0) != 155 && (int)check_.at<uchar>(robot_pos_init_.y, robot_pos_init_.x) == 0)
        {
            done_ = true;
            cv::floodFill(gridmap_.binaryMap(), cv::Point(0,0), cv::Scalar(0));
            std::cout << "finished!!\n";
        }
    }
    done.data = done_;
    done_pub_.publish(done);
    rate_.sleep();
}

void Supervisor::show()
{
    // if (!check_.empty())
    //     cv::imshow("check", check_);
    // cvWaitKey(1);
}