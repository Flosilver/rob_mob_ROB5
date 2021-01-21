#include "trajectory.h"
#include "geometry_msgs/Point.h"

Trajectory::Trajectory(cv::Vec3b col, unsigned int thickness) : list_(),
                                                                color_(col),
                                                                thickness_(thickness) {}

Trajectory::~Trajectory()
{
    list_.clear();
}

// bool Trajectory::setTrajectory(const planification::Checkpoints &checkpoints, const gridmap_2d::GridMap2D &gridmap)
// {
//     list_.clear();
//     unsigned int mx, my;

//     for (const geometry_msgs::Point &pt : checkpoints.response.points.points)
//     {
//         if (gridmap.inMapBounds(pt.x, pt.y))
//         {
//             gridmap.worldToMap(pt.x, pt.y, mx, my);
//             list_.push_back(cv::Point(my, mx));
//         }
//         else
//         {
//             list_.clear();
//             return false;
//         }
//     }
//     return true;
// }

bool Trajectory::show(cv::Mat &dest) const
{
    if (dest.empty())
    {
        return false;
    }
    for (int i = 1; i < list_.size(); i++)
    {
        cv::line(dest, list_[i - 1], list_[i], color_, thickness_);
    }
    return true;
}

void Trajectory::print() const
{
    std::cout << "Trajectoire de checkpoints:\n";
    if (list_.size() == 0)
    {
        std::cout << "empty...\n";
    }
    else
    {
        for (const cv::Point &pt : list_)
        {
            std::cout << pt << ",";
        }
        std::cout << "\n";
    }
}