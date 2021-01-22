#ifndef __RRT_H__
#define __RRT_H__

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"

#include "tree.h"
#include "node.h"
#include <mapping/BinaryMap.h>
#include <planification/ListePoints.h>
#include <planification/Checkpoints.h>

class RRT
{
    ros::Subscriber done_sub_;
    ros::Publisher tree_pub_;
    ros::Subscriber obj_sub_;

    ros::ServiceClient map_client_;

    ros::ServiceServer traj_srv_;

    mapping::BinaryMap map_srv_;
    nav_msgs::OccupancyGrid map_;
    planification::ListePoints traj_;

    Tree ts_; ///< Tree start point
    Tree tg_; ///< Tree goal point
    geometry_msgs::PoseStamped objectiv_;
    bool is_waiting_;
    bool done_;

public:
    RRT(float expand_dist = 1.0f);
    ~RRT();

    bool publish_traj(planification::Checkpoints::Request& req, planification::Checkpoints::Response& res);
    const bool& wait() const { return is_waiting_; };
}

#endif // __RRT_H__