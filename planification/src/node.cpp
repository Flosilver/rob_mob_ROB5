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

Node &Node::operator=(const Node &n)
{
    x_ = n.x_;
    y_ = n.y_;
    neightbours_ = n.neighbours_;
    return *this;
}

Node &Node::newRandNode(const nav_msgs::OccupancyGrid &grid)
{
    Node &new_node();
    int x = 0, y = 0;
    int width = grid.info.width;
    int height = grid.info.height;
    do
    {
        x = rand() % width;
        y = rand() % height;
    } while ((int)grid.data[x * width + y] != FREE);
    new_node.x() = x;
    new_node.y() = y;
    return new_node;
}

float Node::norm(const Node &to) const
{
    return std::sqrt(std::pow(to.x_ - x_, 2) + std::pow(to.y_ - y_));
}

// bool Node::canSee(const Node &n, nav_msgs::OccupancyGrid grid) const
// {
//     float dist = norm(n);
//     int dx = x_ - n.x_;
//     int dy = y_ - n.y_;
//     float e = 0.0f;
//     float e10 = dy / dx;
//     float e01 = -1.0f;
//     float x = x_;
//     float y = y_;

//     for (int i = 0; i < std::max(std::abs(dx), std::abs(dy)); i++)
//     {
//         if (grid.data[((int)x) * grid.info.width + ((int)y)] == OCCUPIED)
//         {
//             return false;
//         }
//         e += e10;
//         if (e >= 0.5)
//         {
//             x += std::abs(dx) < std::abs(dy) ? dx / dist : 0; // we iter on y
//             y += std::abs(dy) < std::abs(dx) ? dy / dist : 0; // we iter on x
//             e += e01;
//         }
//         if (std::abs(dx) < std::abs(dy)) // we iter on y
//         {
//             y += y_ < n.y_ ? 1 : -1;
//         }
//         if (std::abs(dy) < std::abs(dx)) // we iter on x
//         {
//             x += x_ < n.x_ ?
//         }
//     }
// }

bool Node::canSee(const Node &n, nav_msgs::OccupancyGrid grid) const
{
    int dx, dy;
    int x(x_);
    int y(y_);
    uint w = grid.info.width;
    uint h = grid.info.height;

    dx = n.x_ - x_;
    if (dx != 0)
    {
        if (dx > 0)
        {
            dy = n.y_ - y_;
            if (dy != 0)
            {
                if (dy > 0)
                {
                    // 1st cadran
                    if (dx >= dy)
                    {
                        // 1st octant
                        int e(dx);
                        dx = e * 2;
                        dy = dy * 2;

                        do
                        {
                            if (grid.data[x * w + y] == OCCUPIED)
                            {
                                return false;
                            }
                            e -= dy;
                            if (e < 0)
                            {
                                y += 1;
                                e += dx;
                            }
                            x += 1;
                        } while (x == n.x_);
                        return true;
                    }
                    else
                    {
                        // 2nd octant
                        int e(dy);
                        dy = e * 2;
                        dx = dx * 2;

                        do
                        {
                            if (grid.data[x * w + y] == OCCUPIED)
                            {
                                return false;
                            }
                            e -= dx;
                            if (e < 0)
                            {
                                x += 1;
                                e += dy;
                            }
                            y += 1;
                        } while (y == n.y_);
                        return true;
                    }
                }
                else
                {
                    // 4th cadran
                    if (dx >= -dy)
                    {
                        // 8th octant
                        int e(dx);
                        dx = e * 2;
                        dy = dy * 2;

                        do
                        {
                            if (grid.data[x * w + y] == OCCUPIED)
                            {
                                return false;
                            }
                            e += dy;
                            if (e < 0)
                            {
                                y -= 1;
                                e += dx;
                            }
                            x += 1;
                        } while (x == n.x_);
                    }
                }
            }
        }
    }
}

void Node::addNeighbour(Node &n)
{
    neighbours_.push_back(n);
}