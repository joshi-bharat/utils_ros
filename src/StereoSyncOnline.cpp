#include <opencv2/core.hpp>

#include <ros/console.h>

#include "StereoSyncOnline.h"
#include "Utils.h"

StereoSyncOnline::StereoSyncOnline() : nh_private_("~"),
                                       nh_()
{

    ROS_FATAL_STREAM_COND(!nh_private_.getParam("rosbag_filename", rosbag_filename_),
                          "RosBag file param not found");
    if (nh_private_.hasParam("scale"))
        nh_private_.getParam("scale", scale_);
    if (nh_private_.hasParam("compress_image"))
        nh_private_.getParam("compress_image", compress_image_);

    // Wait until time is non-zero and valid: this is because at the ctor level
    // we will be querying for gt pose and/or camera info.

    while (ros::ok() && !ros::Time::now().isValid())
    {
        if (ros::Time::isSimTime())
        {
            ROS_INFO_STREAM_ONCE("Waiting for ROS time to be valid... \n"
                                 << "(Sim Time is enabled; run rosbag with --clock argument)");
        }
        else
        {
            ROS_INFO_STREAM_ONCE("Waiting for ROS time to be valid...");
        }
    }

    bag_.open(rosbag_filename_, rosbag::bagmode::Write);

    //! IMU Subscriber
    static constexpr size_t kMaxImuQueueSize = 1000u;
    imu_subscriber_ = nh_.subscribe("/imu", kMaxImuQueueSize, &StereoSyncOnline::callbackIMU, this);

    //! Sonar Subscriber
    static constexpr size_t kMaxSonarQueueSize = 1000u;
    sonar_subscriber_ = nh_.subscribe("/sonar", kMaxSonarQueueSize, &StereoSyncOnline::callbackSonar, this);

    //! Vision Subscription
    // Subscribe to stereo images. Approx time sync, should be exact though...
    // We set the queue to only 1, since we prefer to drop messages to reach
    // real-time than to be delayed...
    static constexpr size_t kMaxImagesQueueSize = 10u;
    it_ = std::make_unique<image_transport::ImageTransport>(nh_);
    left_img_subscriber_.subscribe(*it_, "/camera0", kMaxImagesQueueSize);
    right_img_subscriber_.subscribe(*it_, "/camera1", kMaxImagesQueueSize);
    static constexpr size_t kMaxImageSynchronizerQueueSize = 10u;
    sync_img_ = std::make_unique<message_filters::Synchronizer<sync_pol_img>>(
        sync_pol_img(kMaxImageSynchronizerQueueSize),
        left_img_subscriber_,
        right_img_subscriber_);

    sync_img_->registerCallback(
        boost::bind(&StereoSyncOnline::callbackStereoImages, this, _1, _2));
}

StereoSyncOnline::~StereoSyncOnline()
{
    ROS_INFO_STREAM("!! BagSync Destructer Called !!");
    bag_.close();
}

void StereoSyncOnline::callbackIMU(const sensor_msgs::ImuConstPtr &imu_msg)
{
    ROS_WARN_ONCE(" !!! Inside IMU Callback !!!");
    bag_.write(imu_topic_, imu_msg->header.stamp, imu_msg);
}

void StereoSyncOnline::callbackStereoImages(const sensor_msgs::ImageConstPtr &left_msg,
                                            const sensor_msgs::ImageConstPtr &right_msg)
{
    ROS_WARN_ONCE(" !!! Inside Stereo Callback !!!");
    using Timestamp = std::uint64_t;
    static Timestamp prev_stamp = 0;
    static uint32_t seq = 0;

    int h = left_msg->height;
    int w = left_msg->width;

    int new_h = int(h * scale_);
    int new_w = int(w * scale_);
    cv::Size size;
    size.height = new_h;
    size.width = new_w;

    const Timestamp &timestamp_left = left_msg->header.stamp.toNSec();
    const Timestamp &timestamp_right = right_msg->header.stamp.toNSec();

    if (timestamp_left > prev_stamp and timestamp_right > prev_stamp)
    {
        Timestamp stamp = std::max(timestamp_right, timestamp_left);
        cv::Mat left_img = Utils::readRosImage(left_msg);
        cv::Mat right_img = Utils::readRosImage(right_msg);

        uint32_t secs = stamp * 1e-9;
        uint32_t n_secs = stamp % 1000000000;
        ros::Time ros_time(secs, n_secs);

        std_msgs::Header header;
        header.seq = seq;
        header.stamp = ros_time;

        if (scale_ != 1.0)
        {
            cv::resize(left_img, left_img, size);
            cv::resize(right_img, right_img, size);
        }

        if (compress_image_)
        {
            sensor_msgs::CompressedImagePtr left_msg, right_msg;
            header.frame_id = "cam0";
            left_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, left_img)
                           .toCompressedImageMsg();
            bag_.write(left_img_topic_ + "/compressed", ros_time, left_msg);

            header.frame_id = "cam1";
            right_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, right_img)
                            .toCompressedImageMsg();
            bag_.write(right_img_topic_ + "/compressed", ros_time, right_msg);
        }
        else
        {
            sensor_msgs::ImagePtr left_msg, right_msg;
            header.frame_id = "cam0";
            left_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, left_img)
                           .toImageMsg();
            bag_.write(left_img_topic_, ros_time, left_msg);

            header.frame_id = "cam1";
            right_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, right_img)
                            .toImageMsg();
            bag_.write(right_img_topic_, ros_time, right_msg);
        }

        prev_stamp = stamp;
    }
    else
    {
        ROS_WARN_STREAM("Left Stamp: " << timestamp_left
                                       << "or Right Stamp: " << timestamp_right
                                       << " not greater than the previous stamp: "
                                       << prev_stamp);
    }
}

void StereoSyncOnline::callbackSonar(const imagenex831l::ProcessedRange::ConstPtr &sonar_msg)
{
    ROS_WARN_ONCE("!!! Inside Sonar Callback !!!");
    bag_.write(sonar_topic_, sonar_msg->header.stamp, sonar_msg);
}
