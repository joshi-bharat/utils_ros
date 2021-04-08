#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include "Definitions.h"
#include "Utils.h"
#include "EurocDataProvider.h"
#include "BagWriter.h"

std::vector<Timestamp> getImageStamps(const std::string rosbag_filename, bool compressed_image = true)
{
    std::vector<Timestamp> stamps;
    rosbag::Bag bag;
    bag.open(rosbag_filename, rosbag::BagMode::Read);

    std::string img_topic;
    if (compressed_image)
        img_topic = "/cam0/image_raw/compressed";
    else
        img_topic = "/cam0/image_raw";

    std::vector<std::string> topics;
    topics.push_back(img_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &msg : view)
    {
        const std::string &msg_topic = msg.getTopic();
        sensor_msgs::CompressedImageConstPtr compressed_img_msg = msg.instantiate<sensor_msgs::CompressedImage>();
        sensor_msgs::ImageConstPtr img_msg = msg.instantiate<sensor_msgs::Image>();

        if (img_msg != nullptr && msg_topic == img_topic && !compressed_image)
        {
            Timestamp stamp = img_msg->header.stamp.toNSec();
            stamps.push_back(stamp);
        }

        if (compressed_image && compressed_img_msg != nullptr && msg_topic == img_topic)
        {
            Timestamp stamp = compressed_img_msg->header.stamp.toNSec();
            stamps.push_back(stamp);
        }
    }
    bag.close();
    return stamps;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "add_svin_pose");

    ros::NodeHandle nh_private("~");
    std::string rosbag_filename;
    std::string svin_result_file;

    ROS_FATAL_STREAM_COND(!nh_private.getParam("rosbag_filename", rosbag_filename),
                          "RosBag file param not found");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("svin_result", svin_result_file),
                          "SVIn result file not found");

    bool compressed_image = true;
    if (nh_private.hasParam("is_compressed_image"))
        nh_private.getParam("is_compressed_image", compressed_image);

    std::vector<Timestamp> camera_stamps = getImageStamps(rosbag_filename, compressed_image);
    ROS_INFO_STREAM("Got " << camera_stamps.size() << " camera messages");

    std::unique_ptr<EurocDataProvider> data_provider = std::make_unique<EurocDataProvider>("");
    data_provider->parseGtData(svin_result_file, ' ', false);

    std::map<Timestamp, gtsam::Pose3> svin_poses = data_provider->ground_truth_;
    ROS_INFO_STREAM("Got " << svin_poses.size() << " SVIn poses");

    BagWriter writer(rosbag_filename, rosbag::bagmode::Append);
    writer.writeGTPose("/svin2/transform_stamped", svin_poses, camera_stamps, true);
}