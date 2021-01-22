#include "node.h"
#include <cmath>
#include <memory>

Node::Node(int x, int y, std::shared_ptr<Node> parent) : x_(x),
                                                         y_(y),
                                                         parent_(parent)
{
}

Node::Node(const Node &n)
{
    x_ = n.x_;
    y_ = n.y_;
    parent_ = n.parent_;
    childs_ = n.childs_;
}

Node::~Node()
{
    childs_.clear();
}

Node &Node::newRandNode(const cv::Mat &env)
{
    Node* new_node;
    int x = 0, y = 0;
    do
    {
        x = rand() % env.rows;
        y = rand() % env.cols;
    } while (env.at<uchar>(x, y) < 255); // OCCUPIED: 0 - FREE: 255
    new_node->x() = x;
    new_node->y() = y;
    return *new_node;
}

float Node::norm(const Tree::Node &to)
{
    return std::sqrt(std::pow(to.x_ - x_, 2) + std::pow(to.y_ - y_));
}

void Node::addChild(Node& n)
{
    childs.push_back(n);
}