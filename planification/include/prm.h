#ifndef __PRM_H__
#define __PRM_H__

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <mapping/BinaryMap.h>
#include <planification/ListePoints.h>
#include <planification/Checkpoints.h>

#include "node.h"
#include <vector>

class PRM
{
    ros::Subscriber done_sub_;
    ros::Publisher graph_pub_;
    ros::Subscriber obj_sub_;

    ros::ServiceClient map_client_;

    ros::ServiceServer traj_srv_;

    //mapping::BinaryMap map_srv_;
    nav_msgs::OccupancyGrid map_;
    planification::ListePoints traj_;

    std::vector<Node> summits_;
    int nb_nodes_;
    geometry_msgs::PoseStamped objectiv_;
    bool mapping_done_;
    bool is_waiting_;
    ros::Rate rate_;

public:
    PRM(int nb_nodes = 10);
    ~PRM();

    bool publishTraj(planification::Checkpoints::Request &req, planification::Checkpoints::Response &res);
    void mappingDoneCallback(const std_msgs::Bool &msg);
    void objectivCallback(const geometry_msgs::PoseStamped &msg);

    const bool &wait() const { return is_waiting_; }
    bool wait() { return is_waiting_; }

    const bool &is_done() const { return mapping_done_; }
    bool is_done() { return mapping_done_; }

    const std::vector<Node> &summits() const { return summits_; }
    std::vector<Node> summits() { return summits_; }
    
    /**
     * @brief Generate the graphe of the RPM. Generate *nb_nodes*
     * random summits in the map. Create neighbours between each node.
     * Send their creation with the graph publisher for the displayer.
     * 
     */
    void generateGraph();

    /**
     * @brief Check if mapping is finished. When it is, the graph
     * is then generated and send for display.
     * 
     * @return true when the mapping is NOT finished.
     * @return false when the mapping is finished.
     */
    bool waitMapping();
};


#endif // __PRM_H__