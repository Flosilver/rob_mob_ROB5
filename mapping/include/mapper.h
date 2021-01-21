#ifndef __MAPPER_H__
#define __MAPPER_H__

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "gridmap2d.h"
#include <mapping/BinaryMap.h>
#include "std_msgs/Bool.h"

class Mapper
{
protected:
    ros::ServiceClient map_client_; ///< Client of the map giver service
    ros::ServiceServer map_serv_; ///< Server of a binary_map service
    ros::Publisher done_pub_; ///< Publisher on a topic specifiing that the the map is fully recovered

    gridmap_2d::GridMap2D gridmap_; ///< Object to treat map recpetion and allow opencv manipulation
    nav_msgs::GetMap map_srv_; ///< Object of the service on map_client_

    bool done_; ///< Flag indicating if the mapping is done
    ros::Rate rate_; ///< Update frequency of the object

public:
    /**
     * @brief Construct a new Mapper object
     * 
     */
    Mapper();

    /**
     * @brief Destroy the Mapper object
     * 
     */
    virtual ~Mapper();

    /**
     * @brief Realize one execution of the object's task.
     * 
     */
    virtual void spin() = 0;

    /**
     * @brief Function called when the bin_map_srv is call by a client.
     * 
     * @param req None
     * @param res OccupancyGrid containing a binarised map of the environment.
     * @return true when the map is not empty.
     * @return false when the map is empty.
     */
    virtual bool publish(mapping::BinaryMap::Request &req, mapping::BinaryMap::Response &res);
};

#endif // __MAPPER_H__