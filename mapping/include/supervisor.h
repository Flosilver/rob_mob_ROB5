#ifndef __SUPERVISOR_H__
#define __SUPERVISOR_H__

#include "mapper.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <opencv2/highgui.hpp>

class Supervisor : public Mapper
{
    cv::Mat check_;
    std::string robot_frame_;
    std::string map_frame_;
    tf2_ros::Buffer* tf_;
    cv::Point robot_pos_init_;

public:
    Supervisor(tf2_ros::Buffer* buffer);
    ~Supervisor();

    void init();
    cv::Point robotPos();
    void odomCallback(const nav_msgs::Odometry &msg);
    virtual void spin();
    void show();
};


#endif // __SUPERVISOR_H__