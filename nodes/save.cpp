#include <ros/ros.h>
#include "datasaver/RGBDSaver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saver");

    staticfusionparser::RGBDSaver saver = staticfusionparser::RGBDSaver();
    ros::spin();

    return 0;
}
