/** @file nodegen.h
    @date 2020/12
    @author Seungwon Song
*/

#ifndef NODEGEN_H
#define NODEGEN_H

#include <ros/ros.h>
#include <sstream>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include "rgbdsaver/node.h"


class nodegen {
private:

    ros::NodeHandle m_nh;/**< Ros node handler */
    ros::Publisher  m_pub_node;/**< Node publisher */

    float m_param_timeSync;
    int   m_node_idx;

    /*sensor data container*/
    std::vector<sensor_msgs::CompressedImage> m_image;
    std::vector<sensor_msgs::CompressedImage> m_depth;
    std::vector<nav_msgs::Odometry>           m_odom;

    void getparam();

    void callback_image(const sensor_msgs::CompressedImage::ConstPtr &msg);

    void callback_depth(const sensor_msgs::CompressedImage::ConstPtr &msg);

    bool update(rgbdsaver::node &nodeOut);

public:
    nodegen();

    ~nodegen();

};

#endif

