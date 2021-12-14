/** @file RGBDSaver.h
    @date 2020/10
    @author Hyungtae Lim
    @brief  Callback RGB and depth, and save them as .png file
*/

#ifndef RGBDSAVER_H
#define RGBDSAVER_H

#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <string>

#include "opencv2/opencv.hpp"

#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <iomanip>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Pose.h>

#include "rgbdsaver/node.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#define MIN_DEPTH 0.3
#define MAX_DEPTH 15.0

enum compressionFormat {
    UNDEFINED = -1, INV_DEPTH
};

struct ConfigHeader {
    // compression format
    compressionFormat format;
    // quantization parameters (used in depth image compression)
    float             depthParam[2];
};

template<class T,size_t N>
boost::array<T,N> stdVec2Boost( const std::vector<T> & v )
{
    //assert(v.size() == N);
    boost::array<T,N> r;
    std::copy( v.begin(), v.end(), r.begin() );
    return r;
}





namespace staticfusionparser {
    class RGBDSaver {
    private:

        ros::NodeHandle nh;/**< Ros node handler */
        ros::Publisher  cloud_pub;

        sensor_msgs::CameraInfo camInfo;
        std::string savedir;
        std::string rgb_order;
        int         init_ts;
        int         final_ts;
        bool        is_initial = true;
        bool        pub_pc;

        bool            flag_caminfoupdate = false;
        Eigen::Matrix3f matK;
        double          range_min          = 0.35;
        double          range_max          = 0.5;

        Eigen::Matrix4f cam2rt;

        void getparam();

        void update_caminfo(sensor_msgs::CameraInfo caminfo);

        void update_range(double min, double max);

        void callback_flag(const std_msgs::Float32::ConstPtr &msg);

        void callback_image(const sensor_msgs::CompressedImage::ConstPtr &msg);

        void callback_depth(const sensor_msgs::CompressedImage::ConstPtr &msg);

        void callback_node(const rgbdsaver::node::ConstPtr& msg);

        Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);

        std::string to_zero_lead(const int value, const unsigned precision) {
            std::ostringstream oss;
            oss << std::setw(precision) << std::setfill('0') << value;
            return oss.str();
        }

        cv::Mat sensorImg2mat(sensor_msgs::Image sensorImg);

        pcl::PointCloud<pcl::PointXYZRGB> getColoredPointCloud(cv::Mat img, cv::Mat depth, int resize=1);

        sensor_msgs::Image::Ptr decodeCompressedDepthImage(const sensor_msgs::CompressedImage &message) {
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

            // Copy message header
            cv_ptr->header = message.header;

            // Assign image encoding
            const size_t      split_pos      = message.format.find(';');
            const std::string image_encoding = message.format.substr(0, split_pos);
            std::string       compression_format;
            // Older version of compressed_depth_image_transport supports only png.
            compression_format = "png";

            cv_ptr->encoding = image_encoding;

            // Decode message data
            if (message.data.size() > sizeof(ConfigHeader)) {

                // Read compression type from stream
                ConfigHeader compressionConfig;
                memcpy(&compressionConfig, &message.data[0], sizeof(compressionConfig));

                // Get compressed image data
                const std::vector <uint8_t> imageData(message.data.begin() + sizeof(compressionConfig), message.data.end());

                // Depth map decoding
                float depthQuantA, depthQuantB;

                // Read quantization parameters
                depthQuantA = compressionConfig.depthParam[0];
                depthQuantB = compressionConfig.depthParam[1];
                if (image_encoding.compare("32FC1") == 0)//(enc::bitDepth(image_encoding) == 32)
                {
                    cv::Mat decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

                    size_t rows = decompressed.rows;
                    size_t cols = decompressed.cols;

                    if ((rows > 0) && (cols > 0)) {
                        cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);

                        // Depth conversion
                        cv::MatIterator_<float>               itDepthImg        = cv_ptr->image.begin<float>(),
                                                              itDepthImg_end    = cv_ptr->image.end<float>();
                        cv::MatConstIterator_<unsigned short> itInvDepthImg     = decompressed.begin<unsigned short>(),
                                                              itInvDepthImg_end = decompressed.end<unsigned short>();

                        for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
                            // check for NaN & max depth
                            if (*itInvDepthImg) {
                                *itDepthImg = depthQuantA / ((float) *itInvDepthImg - depthQuantB);
                            } else {
                                *itDepthImg = std::numeric_limits<float>::quiet_NaN();
                            }
                        }

                        // Publish message to user callback
                        return cv_ptr->toImageMsg();
                    }
                } else {
                    cv_ptr->image = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);

                    size_t rows = cv_ptr->image.rows;
                    size_t cols = cv_ptr->image.cols;

                    if ((rows > 0) && (cols > 0)) {
                        // Publish message to user callback
                        return cv_ptr->toImageMsg();
                    }
                }
            }
            return sensor_msgs::Image::Ptr();
        }


    public:
        RGBDSaver();

        ~RGBDSaver();

    };
}


#endif

