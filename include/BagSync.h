#pragma once

#include <imagenex831l/ProcessedRange.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/opencv.hpp>
/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const> &msg)
    {
        this->signalMessage(msg);
    }
};

class BagSync
{
public:
    BagSync();
    ~BagSync();

public:
    void extractImages();
    void imageCallback(const sensor_msgs::ImageConstPtr &left_img_msg,
                       const sensor_msgs::ImageConstPtr &right_img_msg);
    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &left_img_msg,
                                 const sensor_msgs::CompressedImageConstPtr &right_img_msg);

private:
    std::string left_img_topic_ = "/cam0/image_raw";
    std::string right_img_topic_ = "/cam1/image_raw";
    std::string imu_topic_ = "/imu0";

    std::string rosbag_filename_;
    std::string asl_folder_;
    std::string left_img_folder_;
    std::string right_img_folder_;
    std::string imu_file_;
    std::string timestamps_file_;
    float scale_ = 1.0;
    bool compress_image_ = true;

    ros::NodeHandle nh_private_;

    rosbag::Bag bag_;
};
