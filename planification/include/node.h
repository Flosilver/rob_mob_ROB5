#ifndef __NODE_H__
#define __NODE_H__

// #include <memory>
#include <vector>
#include <cstdlib>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#define OCCUPIED 100
#define FREE 0

class Node
{
    int x_;
    int y_;
    // std::shared_ptr<Node> parent_;
    std::vector<Node> neighbours_;

public:
    Node(int x = 0, int y = 0);
    Node(const Node &n);
    ~Node();
    Node& operator=(const Node& n);
    static Node &newRandNode(const nav_msgs::OccupancyGrid& grid);

    const int &x() const { return x_; }
    int x() { return x_; }
    void x(int x) { x_ = x; }

    const int &y() const { return y_; }
    int y() { return y_; }
    void y(int y) { y_ = y; }

    // std::shared_ptr<const Node> parent() const { return parent_; }
    // std::shared_ptr<Node> parent() { return parent_; }

    const std::vector<Node> neighbours() const { return neighbours_; }
    std::vector<Node> neighbours() { return neighbours_; }

    float norm(const Node &to) const;
    bool canSee(const Node &n, nav_msgs::OccupancyGrid grid) const;
    void addNeighbour(Node &n);

// private:
//     bool cadran(int& x, int& y, int dx, int dy) const;
//     bool octant(int& x, int& y, int dx, int dy, int e) const;
};

#endif // __NODE_H__