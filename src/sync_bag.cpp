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

std::shared_ptr<rosbag::Bag> outbag;
std::string input_bag, output_bag, config_file;
std::string left_img_topic, right_img_topic, imu_topic;
std::string left_compressed_img_topic, right_compressed_img_topic;
bool compress_image;

float scale = 1.0;
int image_height, image_width;
cv::Mat left_map_x, left_map_y, right_map_x, right_map_y;

void imageCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{
    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;

    ROS_INFO_STREAM_ONCE("Inside stereo image callback");

    std_msgs::Header left_header, right_header;
    left_header.stamp = stamp;
    right_header.stamp = stamp;
    left_header.frame_id = left_msg->header.frame_id;
    right_header.frame_id = right_msg->header.frame_id;

    if (scale != 1.0)
    {
        cv::Mat left = Utils::readRosImage(left_msg, false);
        cv::Mat right = Utils::readRosImage(right_msg, false);
        cv::Mat left_undistorted, right_undistorted;
        cv::remap(left, left_undistorted, left_map_x, left_map_y, cv::INTER_LINEAR);
        cv::resize(left_undistorted, left_undistorted, cv::Size(image_width, image_height));
        cv::remap(right, right_undistorted, right_map_x, right_map_y, cv::INTER_LINEAR);
        cv::resize(right_undistorted, right_undistorted, cv::Size(image_width, image_height));

        cv_bridge::CvImage left_cv_image(left_header, "bgr8", left_undistorted);
        cv_bridge::CvImage right_cv_image(right_header, "bgr8", right_undistorted);

        outbag->write(left_img_topic, stamp, left_cv_image.toImageMsg());
        outbag->write(right_img_topic, stamp, right_cv_image.toImageMsg());
    }
    else
    {
        outbag->write(left_img_topic, stamp, left_msg);
        outbag->write(right_img_topic, stamp, right_msg);
    }
}

void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &left_msg, const sensor_msgs::CompressedImageConstPtr &right_msg)
{
    ROS_INFO_STREAM_ONCE("Inside stereo compressed image callback");

    ros::Time stamp = left_msg->header.stamp + (right_msg->header.stamp - left_msg->header.stamp) * 0.5;
    std_msgs::Header left_header, right_header;
    left_header.stamp = stamp;
    right_header.stamp = stamp;
    left_header.frame_id = left_msg->header.frame_id;
    right_header.frame_id = right_msg->header.frame_id;

    if (scale != 1.0)
    {
        cv::Mat left = Utils::readCompressedRosImage(left_msg, false);
        cv::Mat right = Utils::readCompressedRosImage(right_msg, false);
        cv::Mat left_undistorted, right_undistorted;
        cv::remap(left, left_undistorted, left_map_x, left_map_y, cv::INTER_LINEAR);
        cv::resize(left_undistorted, left_undistorted, cv::Size(image_width, image_height));
        cv::remap(right, right_undistorted, right_map_x, right_map_y, cv::INTER_LINEAR);
        cv::resize(right_undistorted, right_undistorted, cv::Size(image_width, image_height));

        cv_bridge::CvImage left_cv_image(left_header, "bgr8", left_undistorted);
        cv_bridge::CvImage right_cv_image(right_header, "bgr8", right_undistorted);

        outbag->write(left_img_topic, stamp, left_cv_image.toImageMsg());
        outbag->write(right_img_topic, stamp, right_cv_image.toImageMsg());
    }
    else
    {
        outbag->write(left_compressed_img_topic, stamp, left_msg);
        outbag->write(right_compressed_img_topic, stamp, right_msg);
    }
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

    if (!nh.hasParam("config"))
    {
        ROS_FATAL_STREAM("config not found");
        exit(1);
    }
    else
    {
        nh.getParam("config", config_file);
    }

    if (nh.hasParam("compressed"))
        nh.getParam("compressed", compress_image);

    if (nh.hasParam("left_image_topic"))
        nh.getParam("left_image_topic", left_img_topic);

    if (nh.hasParam("right_image_topic"))
        nh.getParam("right_image_topic", right_img_topic);

    if (nh.hasParam("imu_topic"))
        nh.getParam("imu_topic", imu_topic);

    if (nh.hasParam("scale"))
        nh.getParam("scale", scale);

    if (compress_image)
    {
        left_compressed_img_topic = left_img_topic + "/compressed";
        right_compressed_img_topic = right_img_topic + "/compressed";
    }

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to intrinsics" << std::endl;
    }

    cv::FileNode left_node = fsSettings["left"];
    cv::FileNode right_node = fsSettings["right"];

    cv::Mat left_distort = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat left_K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat right_distort = cv::Mat::zeros(4, 1, CV_64F);
    cv::Mat right_K = cv::Mat::eye(3, 3, CV_64F);

    Utils::readIntrinsics(left_node, left_K, left_distort);
    Utils::readIntrinsics(right_node, right_K, right_distort);

    image_height = fsSettings["image_height"];
    image_width = fsSettings["image_width"];

    ROS_INFO_STREAM("Left Camera Matrix: " << left_K);
    ROS_INFO_STREAM("Right Camera Matrix: " << right_K);
    ROS_INFO_STREAM("Left Distortion: " << left_distort);
    ROS_INFO_STREAM("Right Distortion: " << right_distort);

    cv::initUndistortRectifyMap(left_K, left_distort, cv::Mat(), left_K, cv::Size(image_width, image_height), CV_32FC1, left_map_x, left_map_y);
    cv::initUndistortRectifyMap(right_K, right_distort, cv::Mat(), right_K, cv::Size(image_width, image_height), CV_32FC1, right_map_x, right_map_y);

    ROS_INFO_STREAM("Orignial height, width: " << image_height << ", " << image_width);

    image_width = static_cast<int>(image_width * scale);
    image_height = static_cast<int>(image_height * scale);

    ROS_INFO_STREAM("New height, width: " << image_height << ", " << image_width);
}

void writeSynchronizedImages()
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
    ROS_INFO_STREAM("Writing synchronized images...");
    writeSynchronizedImages();
    outbag->close();
    return 0;
}