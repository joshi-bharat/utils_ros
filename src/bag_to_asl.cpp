#include "BagSync.h"

#include <opencv2/core.hpp>

#include <ros/console.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <experimental/filesystem>

#include "Utils.h"
#include "Definitions.h"

namespace fs = std::experimental::filesystem;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>
    sync_pol_img;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_pol_compressed_img;

std::string left_img_topic = "/cam0/image_raw";
std::string right_img_topic = "/cam1/image_raw";
std::string imu_topic = "/imu0";

std::string rosbag_filename;
std::string asl_folder;
std::string left_img_folder;
std::string right_img_folder;
std::string imu_filename;
std::string timestamps_filename;
float scale = 1.0;
bool compress_image = true;
rosbag::Bag bag;

void imageCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{
    std::ofstream timestamps_file(timestamps_filename, std::ios::app);

    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    ROS_INFO_STREAM_ONCE("Inside compressed image callback");
    cv::Mat left = Utils::readRosImage(left_msg);
    cv::Mat right = Utils::readRosImage(right_msg);

    std::string left_filename = left_img_folder + "/" + std::to_string(stamp.toNSec()) + ".png";
    std::string right_filename = right_img_folder + "/" + std::to_string(stamp.toNSec()) + ".png";

    cv::imwrite(left_filename, left);
    cv::imwrite(right_filename, right);

    timestamps_file << stamp.toNSec() << std::endl;
    timestamps_file.close();
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &left_msg, const sensor_msgs::CompressedImageConstPtr &right_msg)
{
    std::ofstream timestamps_file(timestamps_filename, std::ios::app);

    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    ROS_INFO_STREAM_ONCE("Inside compressed image callback");
    cv::Mat left = Utils::readCompressedRosImage(left_msg);
    cv::Mat right = Utils::readCompressedRosImage(right_msg);

    std::string left_filename = left_img_folder + "/" + std::to_string(stamp.toNSec()) + ".png";
    std::string right_filename = right_img_folder + "/" + std::to_string(stamp.toNSec()) + ".png";

    cv::imwrite(left_filename, left);
    cv::imwrite(right_filename, right);

    timestamps_file << stamp.toNSec() << std::endl;
    timestamps_file.close();
}

void readParameters(const ros::NodeHandle &nh)
{

    if (!nh.hasParam("bag_filename"))
    {
        ROS_FATAL_STREAM("RosBag file param not found");
        exit(1);
    }
    else
    {
        nh.getParam("bag_filename", rosbag_filename);
    }

    if (!nh.hasParam("asl_folder"))
    {
        ROS_FATAL_STREAM("asl folder not found");
        exit(1);
    }
    else
    {
        nh.getParam("asl_folder", asl_folder);
    }

    if (nh.hasParam("scale"))
        nh.getParam("scale", scale);
    if (nh.hasParam("compressed"))
        nh.getParam("compressed", compress_image);

    if (nh.hasParam("left_image_topic"))
        nh.getParam("left_image_topic", left_img_topic);

    if (nh.hasParam("right_image_topic"))
        nh.getParam("right_image_topic", right_img_topic);

    if (nh.hasParam("imu_topic"))
        nh.getParam("imu_topic", imu_topic);

    if (compress_image)
    {
        left_img_topic += "/compressed";
        right_img_topic += "/compressed";
    }

    left_img_folder = asl_folder + "/mav0/cam0/data";
    if (!fs::exists(left_img_folder))
    {
        fs::create_directories(left_img_folder);
    }
    right_img_folder = asl_folder + "/mav0/cam1/data";
    if (!fs::exists(right_img_folder))
    {
        fs::create_directories(right_img_folder);
    }
    std::string imu_folder = asl_folder + "/mav0/imu0";
    if (!fs::exists(imu_folder))
    {
        fs::create_directories(imu_folder);
    }

    imu_filename = imu_folder + "/data.csv";
    std::ofstream imu_file(imu_filename);
    imu_file << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]" << std::endl;
    imu_file.close();

    timestamps_filename = asl_folder + "/mav0/timestamps.txt";
}

void extractImages()
{

    bag.open(rosbag_filename, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(left_img_topic);
    topics.push_back(right_img_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::ofstream imu_file(imu_filename, std::ios::app);

    BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
    BagSubscriber<sensor_msgs::CompressedImage> l_img_compressed_sub, r_img_compressed_sub;

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::Synchronizer<sync_pol_img> image_sync(sync_pol_img(25), l_img_sub, r_img_sub);
    message_filters::Synchronizer<sync_pol_compressed_img> compressed_image_sync(sync_pol_compressed_img(25),
                                                                                 l_img_compressed_sub, r_img_compressed_sub);

    if (compress_image)
    {
        compressed_image_sync.registerCallback(boost::bind(&compressedImageCallback, _1, _2));
    }
    else
    {
        image_sync.registerCallback(boost::bind(&imageCallback, _1, _2));
    }
    for (rosbag::MessageInstance const m : view)
    {
        if (m.getTopic() == imu_topic)
        {
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            imu_file << imu_msg->header.stamp.toNSec() << "," << imu_msg->angular_velocity.x << "," << imu_msg->angular_velocity.y << "," << imu_msg->angular_velocity.z
                     << "," << imu_msg->linear_acceleration.x << "," << imu_msg->linear_acceleration.y << "," << imu_msg->linear_acceleration.z << std::endl;
        }

        if (m.getTopic() == left_img_topic)
        {
            if (compress_image)
            {
                sensor_msgs::CompressedImage::ConstPtr img_msg = m.instantiate<sensor_msgs::CompressedImage>();
                if (img_msg != nullptr)
                {
                    l_img_compressed_sub.newMessage(img_msg);
                }
            }
            else
            {
                sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
                if (img_msg != nullptr)
                {
                    l_img_sub.newMessage(img_msg);
                }
            }
        }

        if (m.getTopic() == right_img_topic)
        {
            if (compress_image)
            {
                sensor_msgs::CompressedImage::ConstPtr img_msg = m.instantiate<sensor_msgs::CompressedImage>();
                if (img_msg != nullptr)
                {
                    r_img_compressed_sub.newMessage(img_msg);
                }
            }
            else
            {
                sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
                if (img_msg != nullptr)
                {
                    r_img_sub.newMessage(img_msg);
                }
            }
        }
    }
    bag.close();
    imu_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_sync");
    ros::NodeHandle nh_private("~");
    readParameters(nh_private);
    extractImages();
    return 0;
}