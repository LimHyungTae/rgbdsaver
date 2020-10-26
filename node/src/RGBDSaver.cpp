#include "RGBDSaver.h"
using namespace staticfusionparser;



void RGBDSaver::getparam()
{
  m_nh.param<std::string>("savedir", savedir, "/home/shapelim/hdd2/sw_gpslam_rosbag/3f");
}


RGBDSaver::RGBDSaver()
{
  getparam();
  static ros::Subscriber sub2 = m_nh.subscribe<sensor_msgs::CompressedImage>("/rgb/image_raw/compressed",10000,&RGBDSaver::callback_image,this);
  static ros::Subscriber sub3 = m_nh.subscribe<sensor_msgs::CompressedImage>("/depth_to_rgb/image_raw/compressedDepth",10000,&RGBDSaver::callback_depth,this);
  static ros::Subscriber sub_flag = m_nh.subscribe<std_msgs::Float32>("/savetext",10, &RGBDSaver::callback_flag, this);
}

RGBDSaver::~RGBDSaver()
{
}

void RGBDSaver::callback_flag(const std_msgs::Float32::ConstPtr& msg){
  std::cout<<"Flag comes!"<<std::endl;
  std::string filename = savedir + "/rgbd_assoc.txt";
  std::ofstream output(filename.data());
  for (int i=init_ts; i<=final_ts;++i){
    std::string idx = std::to_string(i);
    std::string line = idx + " /rgb/" + idx + ".png " + idx + " /depth/" + idx + ".png\n";
    output<<line;
  }
  output.close();
  std::cout<<"Save complete!"<<std::endl;
}


void RGBDSaver::callback_image(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  cv::Mat colorImg = cv::imdecode(cv::Mat(msg->data),cv::IMREAD_ANYCOLOR);
//  std::cout<<"\033[1;32m"<< colorImg.size <<"  "<<colorImg.type() <<" | "<< CV_8UC3<<" \033[0m"<<std::endl;

  cv::imshow("rgb", colorImg);
  cv::waitKey(1);
  std::string rgb_path = savedir + "/rgb/" + std::to_string(msg->header.seq) + ".png";
  std::cout<<"\033[1;32m Saving"<< rgb_path <<"...\033[0m"<<std::endl;
  cv::cvtColor(colorImg, colorImg, CV_BGR2RGB);
  cv::imwrite(rgb_path, colorImg);

}

void RGBDSaver::callback_depth(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  if (is_initial){
    init_ts = msg->header.seq;
    is_initial = false;
  }
  final_ts = msg->header.seq;
  // parse data
  sensor_msgs::Image::Ptr depthImg = RGBDSaver::decodeCompressedDepthImage(*msg);
  cv::Mat depth = sensorImg2mat(*depthImg);
  // set 16UC1
  cv::Mat depth_16uc1(depth.rows, depth.cols, CV_16UC1);
  // 32FC1 -> 16UC1
  for (uint16_t y=0; y<depth.rows; ++y){
    for (uint16_t x=0; x<depth.cols; ++x){
      float d_value = depth.at<float>(cv::Point(x, y));
      unsigned short d_unsigned_short;
      if (std::isnan(d_value)){
        d_unsigned_short = 0;
      }else{
        d_unsigned_short = (unsigned short)(d_value * 1000); // m -> mm
      }
      depth_16uc1.at<unsigned short>(cv::Point(x, y)) = d_unsigned_short;
    }
  }
  std::string depth_path = savedir + "/depth/" +  std::to_string(msg->header.seq) + ".png";
  std::cout<<"\033[1;33m Saving "<< depth_path <<"...\033[0m"<<std::endl;
  cv::imwrite(depth_path, depth_16uc1);

//  depth_path = savedir + "/depth/" +  std::to_string(msg->header.seq) + "_debug.png";
//  cv::imwrite(depth_path, depth);


}
cv::Mat RGBDSaver::sensorImg2mat(sensor_msgs::Image sensorImg)
{
    static cv_bridge::CvImagePtr cv_ptr;
    cv::Mat mat;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(sensorImg, sensorImg.encoding);
        mat = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return mat;
}

