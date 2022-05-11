#include "BagSync.h"

#include <opencv2/core.hpp>

#include <ros/console.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <experimental/filesystem>

#include "Utils.h"

namespace fs = std::experimental::filesystem;

BagSync::BagSync() : nh_private_("~")
{

    if (!nh_private_.hasParam("bag_filename"))
    {
        ROS_FATAL_STREAM("RosBag file param not found");
        exit(1);
    }
    else
    {
        nh_private_.getParam("bag_filename", rosbag_filename_);
    }

    if (!nh_private_.hasParam("asl_folder"))
    {
        ROS_FATAL_STREAM("asl folder not found");
        exit(1);
    }
    else
    {
        nh_private_.getParam("asl_folder", asl_folder_);
    }

    if (nh_private_.hasParam("scale"))
        nh_private_.getParam("scale", scale_);
    if (nh_private_.hasParam("compressed"))
        nh_private_.getParam("compressed", compress_image_);

    if (nh_private_.hasParam("left_image_topic"))
        nh_private_.getParam("left_image_topic", left_img_topic_);

    if (nh_private_.hasParam("right_image_topic"))
        nh_private_.getParam("right_image_topic", right_img_topic_);

    if (nh_private_.hasParam("imu_topic"))
        nh_private_.getParam("imu_topic", imu_topic_);

    if (compress_image_)
    {
        left_img_topic_ += "/compressed";
        right_img_topic_ += "/compressed";
    }

    left_img_folder_ = asl_folder_ + "/mav0/cam0/data";
    if (!fs::exists(left_img_folder_))
    {
        fs::create_directories(left_img_folder_);
    }
    right_img_folder_ = asl_folder_ + "/mav0/cam1/data";
    if (!fs::exists(right_img_folder_))
    {
        fs::create_directories(right_img_folder_);
    }
    std::string imu_folder = asl_folder_ + "/mav0/imu0";
    if (!fs::exists(imu_folder))
    {
        fs::create_directories(imu_folder);
    }

    imu_file_ = imu_folder + "/data.csv";
    std::ofstream imu_file(imu_file_);
    imu_file << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]" << std::endl;
    imu_file.close();

    timestamps_file_ = asl_folder_ + "/mav0/timestamps.txt";
}

void BagSync::extractImages()
{

    bag_.open(rosbag_filename_, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(imu_topic_);
    topics.push_back(left_img_topic_);
    topics.push_back(right_img_topic_);

    rosbag::View view(bag_, rosbag::TopicQuery(topics));

    std::ofstream imu_file(imu_file_, std::ios::app);

    BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
    BagSubscriber<sensor_msgs::CompressedImage> l_img_compressed_sub, r_img_compressed_sub;

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> image_sync(l_img_sub, r_img_sub, 25);
    message_filters::TimeSynchronizer<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
        compressed_image_sync(l_img_compressed_sub, r_img_compressed_sub, 25);

    if (compress_image_)
    {
        compressed_image_sync.registerCallback(boost::bind(&BagSync::compressedImageCallback, this, _1, _2));
    }
    else
    {
        image_sync.registerCallback(boost::bind(&BagSync::imageCallback, this, _1, _2));
    }
    for (rosbag::MessageInstance const m : view)
    {
        if (m.getTopic() == imu_topic_)
        {
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            imu_file << imu_msg->header.stamp.toNSec() << "," << imu_msg->angular_velocity.x << "," << imu_msg->angular_velocity.y << "," << imu_msg->angular_velocity.z
                     << "," << imu_msg->linear_acceleration.x << "," << imu_msg->linear_acceleration.y << "," << imu_msg->linear_acceleration.z << std::endl;
        }

        if (m.getTopic() == left_img_topic_)
        {
            if (compress_image_)
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

        if (m.getTopic() == right_img_topic_)
        {
            if (compress_image_)
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
    bag_.close();
    imu_file.close();
}

void BagSync::imageCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{
    std::stringstream left_filename, right_filename;
}

void BagSync::compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &left_msg, const sensor_msgs::CompressedImageConstPtr &right_msg)
{
    std::ofstream timestamps_file(timestamps_file_, std::ios::app);

    ROS_INFO_STREAM_ONCE("Inside compressed image callback");
    cv::Mat left = Utils::readCompressedRosImage(left_msg);
    cv::Mat right = Utils::readCompressedRosImage(right_msg);

    std::string left_filename = left_img_folder_ + "/" + std::to_string(left_msg->header.stamp.toNSec()) + ".png";
    std::string right_filename = right_img_folder_ + "/" + std::to_string(right_msg->header.stamp.toNSec()) + ".png";

    cv::imwrite(left_filename, left);
    cv::imwrite(right_filename, right);

    timestamps_file << left_msg->header.stamp.toNSec() << std::endl;
    timestamps_file.close();
}

BagSync::~BagSync()
{
    ROS_INFO_STREAM("!! BagSync Destructer Called !!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_sync");
    BagSync bag_sync;
    bag_sync.extractImages();
    return 0;
}