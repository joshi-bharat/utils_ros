#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <experimental/filesystem>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include "Utils.h"
#include "Definitions.h"
#include "BagSync.h"

namespace fs = std::experimental::filesystem;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
    sync_pol_img;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_pol_compressed_img;

std::string rosbag_filename_, colmap_folder_, left_img_topic_, right_img_topic_;
std::string left_img_folder_, right_img_folder_;
double scale_;
bool compress_image_;
rosbag::Bag bag_;
double last_image_stamp_ = -1;
double delay_ = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{
    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    ROS_INFO_STREAM_ONCE("Inside stereo image callback");
    cv::Mat left = Utils::readRosImage(left_msg, false);
    cv::Mat right = Utils::readRosImage(right_msg, false);

    std::string left_filename = left_img_folder_ + "/" + std::to_string(stamp.toNSec()) + ".png";
    std::string right_filename = right_img_folder_ + "/" + std::to_string(stamp.toNSec()) + ".png";

    cv::imwrite(left_filename, left);
    cv::imwrite(right_filename, right);
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &left_msg, const sensor_msgs::CompressedImageConstPtr &right_msg)
{

    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    if (stamp.toSec() - last_image_stamp_ >= delay_ || last_image_stamp_ == -1)
    {
        ROS_INFO_STREAM_ONCE("Inside stereo compressed image callback");
        cv::Mat left = Utils::readCompressedRosImage(left_msg, false);
        cv::Mat right = Utils::readCompressedRosImage(right_msg, false);

        std::string left_filename = left_img_folder_ + "/" + std::to_string(stamp.toNSec()) + ".png";
        std::string right_filename = right_img_folder_ + "/" + std::to_string(stamp.toNSec()) + ".png";

        cv::imwrite(left_filename, left);
        cv::imwrite(right_filename, right);
        last_image_stamp_ = stamp.toSec();
    }
}

void readParameters(ros::NodeHandle &nh_private_)
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
        nh_private_.getParam("colmap_folder", colmap_folder_);
    }

    if (nh_private_.hasParam("scale"))
        nh_private_.getParam("scale", scale_);
    if (nh_private_.hasParam("compressed"))
        nh_private_.getParam("compressed", compress_image_);

    if (nh_private_.hasParam("left_image_topic"))
        nh_private_.getParam("left_image_topic", left_img_topic_);

    if (nh_private_.hasParam("right_image_topic"))
        nh_private_.getParam("right_image_topic", right_img_topic_);

    if (nh_private_.hasParam("delay"))
        nh_private_.getParam("delay", delay_);

    if (compress_image_)
    {
        left_img_topic_ += "/compressed";
        right_img_topic_ += "/compressed";
    }

    left_img_folder_ = colmap_folder_ + "/left";
    if (!fs::exists(left_img_folder_))
    {
        fs::create_directories(left_img_folder_);
    }
    right_img_folder_ = colmap_folder_ + "/right";
    if (!fs::exists(right_img_folder_))
    {
        fs::create_directories(right_img_folder_);
    }
}

void extractImages()
{

    bag_.open(rosbag_filename_, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(left_img_topic_);
    topics.push_back(right_img_topic_);

    rosbag::View view(bag_, rosbag::TopicQuery(topics));

    BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
    BagSubscriber<sensor_msgs::CompressedImage> l_img_compressed_sub, r_img_compressed_sub;

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::Synchronizer<sync_pol_img> image_sync(sync_pol_img(25), l_img_sub, r_img_sub);
    message_filters::Synchronizer<sync_pol_compressed_img> compressed_image_sync(sync_pol_compressed_img(25),
                                                                                 l_img_compressed_sub, r_img_compressed_sub);

    if (compress_image_)
    {
        compressed_image_sync.registerCallback(boost::bind(&compressedImageCallback, _1, _2));
    }
    else
    {
        image_sync.registerCallback(boost::bind(&imageCallback, _1, _2));
    }
    for (rosbag::MessageInstance const m : view)
    {
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
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_sync");
    ros::NodeHandle nh_private_ = ros::NodeHandle("~");
    readParameters(nh_private_);
    extractImages();
    return 0;
}