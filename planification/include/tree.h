#ifndef __TREE_H__
#define __TREE_H__

#include <vector>
#include <memory>
#include <opencv2/highgui.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "node.h"

class Tree
{
    Node origin_;
    std::vector<Node> all_nodes_;
    float expand_dist_;

public:
    Tree(float expand_dist = 1.0f);
    Tree(int x, int y, float expand_dist = 1.0f);
    ~Tree();

    const Node& connect(const Node &n, const cv::Mat& env);
    const Node &expand(const cv::Mat& env);

    const Node &getClosestNodeTo(const Node &) const;
};


#endif // __TREE_H__