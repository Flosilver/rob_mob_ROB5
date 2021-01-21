#include "tree.h"

#include <cmath>

Tree::Tree(float expand_dist) : expand_dist_(expand_dist)
{
    origin_.x() = 0;
    origin_.y() = 0;
    all_nodes_.push_back(origin_);
}

Tree::Tree(int x, int y, float expand_dist) : expand_dist_(expand_dist)
{
    origin_.x() = x;
    origin_.y() = y;
    all_nodes_.push_back(origin_);
}

Tree::~Tree()
{
    all_nodes_.clear();
}

const Node& Tree::connect(const Node &ngoal, const cv::Mat &env)
{
    // Get closest node to the target
    Node nnear = getClosestNodeTo(ngoal);

    // Create new Node leading to goal
    float angle = std::atan2(ngoal.y() - nnear.y(), ngoal.x() - nnear.x());
    int x = expand_dist_ * std::cos(angle);
    int y = expand_dist_ * std::sin(angle);

    float norm = nnear.norm(ngoal) == 0 ? 1 : nnear.norm(ngoal);
    float dx = (ngoal.x() - nnear.x()) / norm;
    float dy = (ngoal.y() - nnear.y()) / norm;

    int xi = nnear.x();
    int yi = nnear.y();

    // Avoid obstacle while leading to goal
    while (std::floor(xi + dx) == ngoal.x() && std::floor(yi + dy) == ngoal.y())
    {
        xi = std::floor(xi + dx);
        yi = std::floor(yi + dy);
    }

    // Add new node
    std::shared_ptr<Node> parent(new Node(nnear));
    Node& nnew(xi, yi, parent);
    nnear.addChild(nnew);
    all_nodes_.push_back(nnew);

    return nnew;
}

const Node &Tree::expand(const cv::Mat &env)
{
    Node goal = Node::newRandNode(env);
    return connect(goal, env);
}

const Node &Tree::getClosestNodeTo(const Node &n) const
{
    Node nearest = origin_;
    float min = getDist(nearest, n);
    float dist = min;

    for (const Node &node : all_nodes_)
    {
        dist = getDist(node, n);
        if (dist < min)
        {
            min = dist;
            nearest = node;
        }
    }

    return nearest;
}