#include "RGBDSaver.h"

using namespace staticfusionparser;
using namespace std;

void RGBDSaver::getparam() {
    nh.param<std::string>("/savedir", savedir, "/home/shapelim/hdd2/sw_gpslam_rosbag/3f");
    nh.param<std::string>("/rgb_order", rgb_order, "RGB"); // "RGB" or "BGR"
    nh.param<bool>("/pub_pc", pub_pc, true);
    nh.param<bool>("/save_pc", save_pc, true);

    std::vector<double> S;
    std::vector<double> K, R, D;
    if (nh.getParam("camera/S", S)) {
        camInfo.width  = S[0];
        camInfo.height = S[1];
    }
    if (nh.getParam("camera/K", K))
        camInfo.K = stdVec2Boost<double, 9>(K);
    if (nh.getParam("camera/R", R))
        camInfo.R = stdVec2Boost<double, 9>(R);
    if (nh.getParam("camera/D", D))
        camInfo.D = D;
    std::vector<double> tf;
    if(nh.getParam("tf/robot2camera",tf))
    {
        geometry_msgs::Pose geoTF;
        geoTF.position.x = tf[0];
        geoTF.position.y = tf[1];
        geoTF.position.z = tf[2];
        geoTF.orientation.x = tf[3];
        geoTF.orientation.y = tf[4];
        geoTF.orientation.z = tf[5];
        geoTF.orientation.w = tf[6];
        cam2rt = geoPose2eigen(geoTF);
    }
    std::cout<< camInfo <<std::endl;
    std::cout<< cam2rt <<std::endl;
    update_caminfo(camInfo);
    update_range(MIN_DEPTH, MAX_DEPTH);
}

void RGBDSaver::update_caminfo(sensor_msgs::CameraInfo caminfo) {
    flag_caminfoupdate = true;
    matK << caminfo.K[0], caminfo.K[1], caminfo.K[2],
            caminfo.K[3], caminfo.K[4], caminfo.K[5],
            caminfo.K[6], caminfo.K[7], caminfo.K[8];
}

void RGBDSaver::update_range(double min, double max) {
    range_min = min;
    range_max = max;
}

pcl::PointCloud<pcl::PointXYZRGB> RGBDSaver::getColoredPointCloud(cv::Mat img, cv::Mat depth, int resize) {
    pcl::PointCloud<pcl::PointXYZRGB> result;

    if (!flag_caminfoupdate) {
        std::cout << "Error : no K matrix" << std::endl;
        return result;
    }

    cv::Mat depthadjust;
    if (img.size().width == depth.size().width)
        depthadjust             = depth.clone();
    else
        cv::resize(depth, depthadjust, img.size());

    cv::Mat         imgresize;
    cv::Mat         depthresize;
    Eigen::Matrix3f K;
    if(resize == 1)
    {
        imgresize = img.clone();
        depthresize = depthadjust.clone();
        K = matK;
    }
    else
    { // ToDo: Suppor resize model
//        cv::resize(img,imgresize,cv::Size(img.size().width/resize,img.size().height/resize));
//        cv::resize(depthresize,depthajust, imgresize.size());
//        K << matK(0,0) /resize, matK(0,1) /resize, matK(0,2) /resize,
//             matK(1,0) /resize, matK(1,1) /resize, matK(1,2) /resize,
//             matK(2,0) /resize, matK(2,1) /resize, matK(2,2);
    }

    std::vector<float> pts;
    pts.reserve(depthresize.size().height * depthresize.size().width * 2);

    for (int           y        = 0; y < depthresize.size().height; y++) {
        for (int x = 0; x < depthresize.size().width; x++) {
            pts.push_back(x);
            pts.push_back(y);
//            std::cout << depthresize.at<float>(y, x) <<std::endl;
        }
    }
    int                pts_size = pts.size() / 2;
    Eigen::MatrixXf pts1      = Eigen::Map<Eigen::Matrix<float, 2, Eigen::Dynamic> >(pts.data(), 2, pts_size);
    Eigen::MatrixXf pts1_ones = Eigen::Matrix<float, 1, Eigen::Dynamic>(1, pts_size);
    pts1_ones.setOnes();
    Eigen::MatrixXf pts2 = Eigen::Matrix<float, 3, Eigen::Dynamic>(3, pts_size);
    pts2 << pts1,
            pts1_ones;
    Eigen::MatrixXf pts2_Kinv = K.inverse() * pts2;

    std::vector<float>     depths;
    depths.reserve(depthresize.size().height * depthresize.size().width);

    for (int               y  = 0; y < depthresize.size().height; y++) {
        for (int x = 0; x < depthresize.size().width; x++) {
            if (depthresize.type() == CV_16UC1) {
                depths.push_back((float) depthresize.at<unsigned short>(y, x) / 1000);
            } else {
                depths.push_back(depthresize.at<float>(y, x));
            }
        }
    }
    std::vector<cv::Vec3b> colors;
    for (int               y  = 0; y < imgresize.size().height; y++) {
        for (int x = 0; x < imgresize.size().width; x++) {
            colors.push_back(imgresize.at<cv::Vec3b>(y, x));
        }
    }

    Eigen::MatrixXf deps1 = Eigen::Map<Eigen::Matrix<float, 1, Eigen::Dynamic> >(depths.data(), 1, pts_size);
    Eigen::MatrixXf xyz(3, pts_size);
    xyz.row(0) = pts2_Kinv.row(0).cwiseProduct(deps1);
    xyz.row(1) = pts2_Kinv.row(1).cwiseProduct(deps1);
    xyz.row(2) = deps1;

    for (int i = 0; i < xyz.cols(); i++) {
        pcl::PointXYZRGB tmp_xyz;
        tmp_xyz.x = xyz(0, i);
        tmp_xyz.y = xyz(1, i);
        tmp_xyz.z = xyz(2, i);
        tmp_xyz.r = colors.at(i)[2];
        tmp_xyz.g = colors.at(i)[1];
        tmp_xyz.b = colors.at(i)[0];
        if ( std::isnan(tmp_xyz.x) || std::isnan(tmp_xyz.y) || std::isnan(tmp_xyz.z) ) { continue; }
        if (xyz(2, i) > range_min && xyz(2, i) < range_max)
            result.push_back(tmp_xyz);
    }
    return result;
}

RGBDSaver::RGBDSaver() {
    getparam();

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 100);

    static ros::Subscriber sub1     = nh.subscribe<rgbdsaver::node>("/sync_node", 10000, &RGBDSaver::callback_node, this);
    static ros::Subscriber sub2     = nh.subscribe<sensor_msgs::CompressedImage>("/saver/image", 10000, &RGBDSaver::callback_image, this);
    static ros::Subscriber sub3     = nh.subscribe<sensor_msgs::CompressedImage>("saver/depth", 10000, &RGBDSaver::callback_depth, this);
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
    sensor_msgs::Image::Ptr depthImg = RGBDSaver::decodeCompressedDepthImage(*msg);
    cv::Mat                 depth    = sensorImg2mat(*depthImg);
    // set 16UC1
    cv::Mat                 depth_16uc1(depth.rows, depth.cols, CV_16UC1);
    // 32FC1 -> 16UC1
    for (uint16_t           y        = 0; y < depth.rows; ++y) {
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

    std::string depth_path = savedir + "/depth/" + std::to_string(msg->header.seq) + ".png";
    std::cout << "\033[1;33m Saving " << depth_path << "...\033[0m" << std::endl;
    cv::imwrite(depth_path, depth_16uc1);
}

void RGBDSaver::callback_node(const rgbdsaver::node::ConstPtr &msg) {
    std::cout << "Node in!!!" << std::endl;

    cv::Mat colorImg = cv::imdecode(cv::Mat(msg->image.data), cv::IMREAD_ANYCOLOR);
    sensor_msgs::Image::Ptr depthImg = RGBDSaver::decodeCompressedDepthImage(msg->depth);
    cv::Mat                 depth    = sensorImg2mat(*depthImg);

    cout << colorImg.cols << ", " << colorImg.rows << " versus ";
    cout << depth.cols << ", " << depth.rows <<endl;

    pcl::PointCloud<pcl::PointXYZRGB> colored_pc;
    pcl::transformPointCloud(getColoredPointCloud(colorImg, depth), colored_pc, cam2rt);


    static int count = 0;
    if (pub_pc) {
        std::cout<< count << "th pc is published!" << std::endl;
        sensor_msgs::PointCloud2 cloud_ROS;
        pcl::toROSMsg(colored_pc, cloud_ROS);
        cloud_ROS.header.stamp    = ros::Time::now();
        cloud_ROS.header.frame_id = "/map";
        cloud_pub.publish(cloud_ROS);
    }

    if (save_pc) {
        static int count_for_save = 0;
        if (++count % 5 == 0) {
            std::string abs_save_path = (boost::format("%s/%06d.pcd") % savedir % count_for_save).str();
            pcl::io::savePCDFileASCII(abs_save_path, colored_pc);
            count_for_save++;
        }
    }
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


Eigen::Matrix4f RGBDSaver::geoPose2eigen(geometry_msgs::Pose geoPose)
{
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf orientation(geoPose.orientation.w, geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z);
    Eigen::Matrix3f rot = orientation.toRotationMatrix();
    result.block<3, 3>(0, 0) = rot;
//    result(0,0) = m[0][0];
//    result(0,1) = m[0][1];
//    result(0,2) = m[0][2];
//    result(1,0) = m[1][0];
//    result(1,1) = m[1][1];
//    result(1,2) = m[1][2];
//    result(2,0) = m[2][0];
//    result(2,1) = m[2][1];
//    result(2,2) = m[2][2];
    result(3,3) = 1;

    result(0,3) = geoPose.position.x;
    result(1,3) = geoPose.position.y;
    result(2,3) = geoPose.position.z;

    return result;
}

