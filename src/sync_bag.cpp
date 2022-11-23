#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <experimental/filesystem>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

#include <cv_bridge/cv_bridge.h>

#include "Utils.h"
#include "Definitions.h"
#include "BagSync.h"

namespace fs = std::experimental::filesystem;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
    sync_pol_img;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>
    sync_pol_compressed_img;

std::string input_bag, output_bag;
std::string left_img_topic, right_img_topic, imu_topic;
std::string left_compressed_img_topic, right_compressed_img_topic;
bool compress_image;
std::shared_ptr<rosbag::Bag> outbag;

void imageCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{
    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    ROS_INFO_STREAM_ONCE("Inside stereo image callback");
    cv::Mat left = Utils::readRosImage(left_msg, true);
    cv::Mat right = Utils::readRosImage(right_msg, true);

    std_msgs::Header left_header, right_header;
    left_header.stamp = stamp;
    right_header.stamp = stamp;
    left_header.frame_id = left_msg->header.frame_id;
    right_header.frame_id = right_msg->header.frame_id;

    cv_bridge::CvImage left_image(left_header, "mono8", left);
    cv_bridge::CvImage right_image(right_header, "mono8", right);

    outbag->write(left_img_topic, stamp, left_image.toImageMsg());
    outbag->write(right_img_topic, stamp, right_image.toImageMsg());
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &left_msg, const sensor_msgs::CompressedImageConstPtr &right_msg)
{

    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    ROS_INFO_STREAM_ONCE("Inside stereo compressed image callback");
    cv::Mat left = Utils::readCompressedRosImage(left_msg, true);
    cv::Mat right = Utils::readCompressedRosImage(right_msg, true);

    std_msgs::Header left_header, right_header;
    left_header.stamp = stamp;
    right_header.stamp = stamp;
    left_header.frame_id = left_msg->header.frame_id;
    right_header.frame_id = right_msg->header.frame_id;

    cv_bridge::CvImage left_image(left_header, "mono8", left);
    cv_bridge::CvImage right_image(right_header, "mono8", right);

    outbag->write(left_img_topic, stamp, left_image.toImageMsg());
    outbag->write(right_img_topic, stamp, right_image.toImageMsg());
}

void readParameters(const ros::NodeHandle &nh)
{

    if (!nh.hasParam("input_bag"))
    {
        ROS_FATAL_STREAM("input bag file param not found");
        exit(1);
    }
    else
    {
        nh.getParam("input_bag", input_bag);
    }

    if (!nh.hasParam("output_bag"))
    {
        ROS_FATAL_STREAM("output_bag not found");
        exit(1);
    }
    else
    {
        nh.getParam("output_bag", output_bag);
    }

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
        left_compressed_img_topic = left_img_topic + "/compressed";
        right_compressed_img_topic = right_img_topic + "/compressed";
    }
}

void extractImages()
{

    rosbag::Bag inbag;
    inbag.open(input_bag, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    if (compress_image)
    {
        topics.push_back(left_compressed_img_topic);
        topics.push_back(right_compressed_img_topic);
    }
    else
    {

        topics.push_back(left_img_topic);
        topics.push_back(right_img_topic);
    }
    topics.push_back(imu_topic);

    rosbag::View view(inbag, rosbag::TopicQuery(topics));

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
        if (m.getTopic() == left_compressed_img_topic)
        {

            sensor_msgs::CompressedImage::ConstPtr img_msg = m.instantiate<sensor_msgs::CompressedImage>();
            if (img_msg != nullptr)
            {
                l_img_compressed_sub.newMessage(img_msg);
            }
        }

        if (m.getTopic() == right_compressed_img_topic)
        {
            sensor_msgs::CompressedImage::ConstPtr img_msg = m.instantiate<sensor_msgs::CompressedImage>();
            if (img_msg != nullptr)
            {
                r_img_compressed_sub.newMessage(img_msg);
            }
        }

        if (m.getTopic() == left_img_topic)
        {
            sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            if (img_msg != nullptr)
            {
                l_img_sub.newMessage(img_msg);
            }
        }

        if (m.getTopic() == right_img_topic)
        {
            sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            if (img_msg != nullptr)
            {
                r_img_sub.newMessage(img_msg);
            }
        }

        if (m.getTopic() == imu_topic)
        {
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg != nullptr)
            {
                outbag->write(imu_topic, imu_msg->header.stamp, imu_msg);
            }
        }
    }
    inbag.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_sync");
    ros::NodeHandle nh_private_ = ros::NodeHandle("~");
    readParameters(nh_private_);
    ROS_INFO_STREAM("Input bag: " << input_bag);
    ROS_INFO_STREAM("Output bag: " << output_bag);
    outbag.reset(new rosbag::Bag(output_bag, rosbag::bagmode::Write));
    ROS_INFO_STREAM("Extracting images...");
    extractImages();
    sleep(10);
    outbag->close();
    return 0;
}