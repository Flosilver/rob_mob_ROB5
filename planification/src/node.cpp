#include "node.h"
#include <cmath>
// #include <memory>

Node::Node(int x, int y) : x_(x),
                           y_(y),
{
}

Node::Node(const Node &n)
{
    x_ = n.x_;
    y_ = n.y_;
    neighbours_ = n.neighbours_;
}

Node::~Node()
{
    neighbours_.clear();
}

Node& Node::operator=(const Node& n)
{
    x_ = n.x_;
    y_ = n.y_;
    neightbours_ = n.neighbours_;
    return *this;
}

Node &Node::newRandNode(const nav_msgs::OccupancyGrid& grid)
{
    Node *new_node;
    int x = 0, y = 0;
    int width = grid.info.width;
    int height = grid.info.height;
    do
    {
        x = rand() % width;
        y = rand() % height;
    } while ((int)grid.data[x*width + y] != FREE);
    new_node->x() = x;
    new_node->y() = y;
    return *new_node;
}

float Node::norm(const Node &to)
{
    return std::sqrt(std::pow(to.x_ - x_, 2) + std::pow(to.y_ - y_));
}

void Node::addNeighbour(Node &n)
{
    neighbours_.push_back(n);
}