#include "loader.h"

#include <cstdlib>
#include <memory>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_loader");
    ros::NodeHandle n;
    std::shared_ptr<Mapper> loader(new Loader());

    while (ros::ok())
    {
        loader->spin();
        ros::spinOnce();
    }

    return 0;
}