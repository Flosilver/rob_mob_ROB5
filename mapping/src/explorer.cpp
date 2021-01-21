#include "explorer.h"

#include <string>
#include <cmath>

Explorer::Explorer() : k1_(0.0f),
                       k2_(0.0f),
                       speed_(0.0f),
                       d0_(0.0f),
                       scan_sub_(),
                       cmd_pub_(),
                       done_sub_(),
                       done_(false),
                       stoped_(false)
{
    ros::NodeHandle n("~");

    // get params
    speed_ = n.param("speed", 1.0f);
    k1_ = n.param("k1", 1.0f);
    k2_ = n.param("k2", 1.0f);
    d0_ = n.param("distance", 1.0f);

    // get topic and service names
    std::string scan_topic(n.param<std::string>("scan_topic", "/scan"));
    std::string cmd_topic(n.param<std::string>("cmd_topic", "/cmd_vel"));
    std::string done_topic(n.param<std::string>("done_topic", "/mapping_done"));

    // Initialize subscriber and publisher
    scan_sub_ = n.subscribe(scan_topic, 1, &Explorer::scanCallback, this);
    cmd_pub_ = n.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    done_sub_ = n.subscribe(done_topic, 1, &Explorer::doneCallback, this);
}

Explorer::~Explorer() {}

void Explorer::scanCallback(const sensor_msgs::LaserScan &msg)
{
    if (!done_)
    {
        size_t i_min(0);
        size_t ranges_len(msg.ranges.size());

        // Find the index of the closest obstacle
        for (size_t i = 1; i < ranges_len; i++)
        {
            if (msg.ranges.at(i) < msg.ranges.at(i_min))
                i_min = i;
        }

        // Indexes for wall detection
        size_t index_i, index_j;
        index_i = 0;
        index_j = index_from_angle(-M_PI / 4, msg);

        // Get points of the obstacle to dertermine its direction
        float xi(0.0f), yi(0.0f), xj(0.0f), yj(0.0f);
        compute_pos(msg, index_i, xi, yi);
        compute_pos(msg, index_j, xj, yj);

        // Create the command
        geometry_msgs::Twist cmd;

        // Linear
        cmd.linear.x = speed_;

        // Angular
        float dist = msg.ranges.at(i_min);
        float tan((yi - yj) / (xi - xj));
        float angle(std::atan2(yi - yj, xi - xj));
        cmd.angular.z = k1_ * (dist - d0_) / (speed_ * std::cos(angle)) - k2_ * -tan;
        cmd_pub_.publish(cmd);
    }
    else if (!stoped_)
    {
        geometry_msgs::Twist cmd;
        cmd_pub_.publish(cmd);
        stoped_ = true;
    }
}

void Explorer::doneCallback(const std_msgs::Bool &msg)
{
    done_ = msg.data;
}

void Explorer::compute_pos(const sensor_msgs::LaserScan &msg, const size_t index, float &x, float &y)
{
    float angle(msg.angle_min + msg.angle_increment * index);
    float dist(msg.ranges.at(index) < msg.range_max ? msg.ranges.at(index) : msg.range_max);
    x = dist * std::cos(angle);
    y = dist * std::sin(angle);
}

size_t Explorer::index_from_angle(float angle, const sensor_msgs::LaserScan &msg)
{
    return (angle - msg.angle_min) / msg.angle_increment;
}