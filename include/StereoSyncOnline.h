#ifndef STEROSYNCONLINE_H
#define STEROSYNCONLINE_H

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

class StereoSyncOnline
{
public:
    StereoSyncOnline();
    ~StereoSyncOnline();

public:
    void callbackIMU(const sensor_msgs::ImuConstPtr &imu_msg);
    void callbackStereoImages(const sensor_msgs::ImageConstPtr &left_msg,
                              const sensor_msgs::ImageConstPtr &right_msg);
    void callbackSonar(const imagenex831l::ProcessedRange::ConstPtr &sonar_msg);

private:
    std::string left_img_topic_ = "/cam0/image_raw";
    std::string right_img_topic_ = "/cam1/image_raw";
    std::string imu_topic_ = "/imu0";
    std::string sonar_topic_ = "/sonar0";

    std::string rosbag_filename_;
    float scale_ = 1.0;
    bool compress_image_ = true;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Define image transport for this and derived classes.
    std::unique_ptr<image_transport::ImageTransport> it_;

    // Message filters and to sync stereo images
    typedef image_transport::SubscriberFilter ImageSubscriber;
    ImageSubscriber left_img_subscriber_;
    ImageSubscriber right_img_subscriber_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image>
        sync_pol_img;
    std::unique_ptr<message_filters::Synchronizer<sync_pol_img>> sync_img_;

    // Define subscriber for IMU data
    ros::Subscriber imu_subscriber_;
    ros::Subscriber sonar_subscriber_;

    rosbag::Bag bag_;
};

#endif // STEROSYNCONLINE_H