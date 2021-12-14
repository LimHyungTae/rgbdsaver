#include "nodegen.h"

std::vector<std::pair<int, double> > timediff(std::vector<std::pair<int, std_msgs::Header>> times);

nodegen::nodegen() {
    getparam();
    static ros::Subscriber sub2 = m_nh.subscribe<sensor_msgs::CompressedImage>("/image", 10000, &nodegen::callback_image, this);
    static ros::Subscriber sub3 = m_nh.subscribe<sensor_msgs::CompressedImage>("/depth", 10000, &nodegen::callback_depth, this);
    m_pub_node = m_nh.advertise<rgbdsaver::node>("/nodegen/node", 1000);
}

nodegen::~nodegen() {
}

void nodegen::callback_image(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    m_image.push_back(*msg);
    rgbdsaver::node nodeOut;
    if (update(nodeOut))
        m_pub_node.publish(nodeOut);
}

void nodegen::callback_depth(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    m_depth.push_back(*msg);
    rgbdsaver::node nodeOut;
    if (update(nodeOut))
        m_pub_node.publish(nodeOut);
}

bool nodegen::update(rgbdsaver::node &nodeOut) //If all data is prepared, generate combined node.
{
    static int node_idx = 0;
    while (!m_image.empty() && !m_depth.empty()) {
        double              timeRef = m_image.front().header.stamp.toSec();
        std::vector<double> times{
                m_image.front().header.stamp.toSec(),
                m_depth.front().header.stamp.toSec()};
        auto                idxMM   = std::minmax_element(times.begin(), times.end());
        if (*idxMM.second - *idxMM.first > m_param_timeSync) {
            if (idxMM.first - times.begin() == 0) m_image.erase(m_image.begin());
            else if (idxMM.first - times.begin() == 1) m_depth.erase(m_depth.begin());
        } else {
            std::cout << "Sync complete!" << std::endl;
            nodeOut.image         = m_image.front();
            nodeOut.depth         = m_depth.front();
            m_image.erase(m_image.begin());
            m_depth.erase(m_depth.begin());
            return true;
        }
    }
    return false;
}

void nodegen::getparam() {
    m_nh.param<float>("node/timeSync", m_param_timeSync, 0.1);
}
