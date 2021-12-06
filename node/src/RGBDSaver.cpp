#include "RGBDSaver.h"

using namespace staticfusionparser;


void RGBDSaver::getparam() {
    nh.param<std::string>("/savedir", savedir, "/home/shapelim/hdd2/sw_gpslam_rosbag/3f");
    nh.param<std::string>("/rgb_msgname", rgb_msgname, "/rgb/compressed");
    nh.param<std::string>("/depth_msgname", depth_msgname, "/depth/compressed");
    nh.param<std::string>("/rgb_order", rgb_order, "RGB"); // "RGB" or "BGR"

    nh.param<bool>("/depth2pc", depth2pc, true); // "RGB" or "BGR"

    nh.param<float>("/fx", fx, 612.0);
    nh.param<float>("/fy", fy, 612.0);
    nh.param<float>("/cx", cx, 637.6094);
    nh.param<float>("/cy", cy, 369.3013);
}


RGBDSaver::RGBDSaver() {
    getparam();
    std::cout << "Target RGB msg name: " << rgb_msgname << std::endl;
    std::cout << "Target Depth msg name: " << depth_msgname << std::endl;

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("depth2pc", 100);

    static ros::Subscriber sub2     = nh.subscribe<sensor_msgs::CompressedImage>(rgb_msgname, 10000, &RGBDSaver::callback_image, this);
    static ros::Subscriber sub3     = nh.subscribe<sensor_msgs::CompressedImage>(depth_msgname, 10000, &RGBDSaver::callback_depth, this);
    static ros::Subscriber sub_flag = nh.subscribe<std_msgs::Float32>("/savetext", 10, &RGBDSaver::callback_flag, this);
}

RGBDSaver::~RGBDSaver() {
}

void RGBDSaver::callback_flag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "Flag comes!" << std::endl;
    std::string   filename = savedir + "/rgbd_assoc.txt";
    std::ofstream output(filename.data());
    for (int      i        = init_ts; i <= final_ts; ++i) {
        std::string idx  = std::to_string(i);
        std::string line = idx + " /rgb/" + idx + ".png " + idx + " /depth/" + idx + ".png\n";
        output << line;
    }
    output.close();
    std::cout << "Save complete!" << std::endl;
}


void RGBDSaver::callback_image(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    cv::Mat colorImg = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_ANYCOLOR);
//  std::cout<<"\033[1;32m"<< colorImg.size <<"  "<<colorImg.type() <<" | "<< CV_8UC3<<" \033[0m"<<std::endl;

    cv::imshow("rgb", colorImg);
    cv::waitKey(1);
    std::string rgb_path = savedir + "/rgb/" + std::to_string(msg->header.seq) + ".png";
    std::cout << "\033[1;32m Saving" << rgb_path << "...\033[0m" << std::endl;
    if (rgb_order == "RGB") {
        cv::cvtColor(colorImg, colorImg, CV_BGR2RGB);
    }
    cv::imwrite(rgb_path, colorImg);

}

void RGBDSaver::callback_depth(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    if (is_initial) {
        init_ts    = msg->header.seq;
        is_initial = false;
    }
    final_ts = msg->header.seq;
    // parse data
    sensor_msgs::Image::Ptr depthImg   = RGBDSaver::decodeCompressedDepthImage(*msg);
    cv::Mat                 depth      = sensorImg2mat(*depthImg);
    // set 16UC1
    cv::Mat                 depth_16uc1(depth.rows, depth.cols, CV_16UC1);
    // 32FC1 -> 16UC1
    for (uint16_t           y          = 0; y < depth.rows; ++y) {
        for (uint16_t x = 0; x < depth.cols; ++x) {
            float          d_value = depth.at<float>(cv::Point(x, y));
            unsigned short d_unsigned_short;
            if (std::isnan(d_value)) {
                d_unsigned_short = 0;
            } else {
                d_unsigned_short = (unsigned short) (d_value * 1000); // m -> mm
            }
            depth_16uc1.at<unsigned short>(cv::Point(x, y)) = d_unsigned_short;
        }
    }

    if (depth2pc) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        mat2pcd(depth, cloud);

        sensor_msgs::PointCloud2 cloud_ROS;

        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.stamp    = ros::Time::now();
        cloud_ROS.header.frame_id = "/map";
        cloud_pub.publish(cloud_ROS);
    }

    std::string             depth_path = savedir + "/depth/" + std::to_string(msg->header.seq) + ".png";
    std::cout << "\033[1;33m Saving " << depth_path << "...\033[0m" << std::endl;
    cv::imwrite(depth_path, depth_16uc1);
}

void RGBDSaver::mat2pcd(const cv::Mat& depth, pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::PointXYZ pt;
    for (uint16_t           y          = 0; y < depth.rows; ++y) {
        for (uint16_t x = 0; x < depth.cols; ++x) {
            float          d_value = depth.at<float>(cv::Point(x, y));
            if (std::isnan(d_value)) {
                continue;
            } else {
                uvd2xyz(x, y, d_value, pt);
                cloud.points.emplace_back(pt);
            }
        }
    }
}

void RGBDSaver::uvd2xyz(float u, float v, float d, pcl::PointXYZ& pt) {
    //d: meter
    float x_over_z = (cx - u) / fx;
    float y_over_z = (cy - v) / fy;
    float z = d / sqrt(1. + pow(x_over_z,2) + pow(y_over_z, 2));
    float x = x_over_z * z;
    float y = y_over_z * z;

    pt.x = x;
    pt.y = y;
    pt.z = z;
}

cv::Mat RGBDSaver::sensorImg2mat(sensor_msgs::Image sensorImg) {
    static cv_bridge::CvImagePtr cv_ptr;
    cv::Mat                      mat;
    try {
        cv_ptr = cv_bridge::toCvCopy(sensorImg, sensorImg.encoding);
        mat    = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return mat;
}

