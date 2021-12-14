
#include <ros/ros.h>
#include "nodegen/nodegen.h"

int main(int argc, char **argv)
{


    ros::init(argc, argv, "url_nodegen");

    nodegen nodegen_class = nodegen();
    ros::spin();

    return 0;
}
