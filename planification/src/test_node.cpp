#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"
#include <mapping/BinaryMap.h>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>

#include "prm.h"

bool done = false;

void doneCallback(const std_msgs::Bool& msg)
{
    done = msg.data;
}

int main(int argc, char** argv)
{  
    std::srand(time(NULL));
    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    /*ros::ServiceClient map_client = n.serviceClient<mapping::BinaryMap>("/binary_map");
    ros::Subscriber done_sub = n.subscribe("/mapping_done", 1, doneCallback);

    mapping::BinaryMap map_srv;
    nav_msgs::OccupancyGrid map;
    // std::vector<unsigned char> data;

    while (!done) { ros::spinOnce(); }
    while (!map_client.call(map_srv)) { ros::spinOnce(); }
    map = map_srv.response.map;
    // data = map.data;

    std::cout << (int)map.data[0] << std::endl;*/

    PRM prm(20);
    std::cout << "start!\n";

    while (prm.waitMapping())
    {
        // std::cout << "piou\n";
        ros::spinOnce();
    }

    std::cout << "the end\n";
    
    // fonction de génération de trajectoire en fonction de l'objectif et 
    // de la position du robot.
    // TODO 

    return 0;
}