#include <ros/console.h>
#include <ros/ros.h>

#include "EurocDataProvider.h"
#include "BagWriter.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "asl_to_bag");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string dataset_path;
    std::string bag_filename;
    if (!nh_private.getParam("dataset_path", dataset_path))
        ROS_FATAL_STREAM("Dataset path not provided\n");

    if (!nh_private.getParam("bagfile", bag_filename))
        ROS_FATAL_STREAM("Ros bag file name not provided\n");

    std::string cam0_topic = "/cam0/image_raw";
    std::string cam1_topic = "/cam1/image_raw";
    std::string imu_topic = "/imu0";

    float scale = 1.0;
    if(nh_private.hasParam("scale"))
        nh.getParam("scale", scale);

    bool use_stereo = true;
    if(nh_private.hasParam("stereo"))
        nh.getParam("stereo", use_stereo);

    bool use_imu = true;
    if (nh_private.hasParam("use_imu"))
        nh_private.getParam("use_imu", use_imu);

    bool use_gt = true;
    if (nh_private.hasParam("use_gt"))
        nh_private.getParam("use_gt", use_gt);

    std::unique_ptr<EurocDataProvider> data_provider = std::make_unique<EurocDataProvider>(dataset_path);
    // ROS_INFO_STREAM("Writing to Bag File: " << bag_filename);
    std::unique_ptr<BagWriter> bag_writer = std::make_unique<BagWriter>(bag_filename);

    data_provider->parseLeftCam();
    if(use_stereo)
        data_provider->parseRightCam();
    if(use_imu)
        data_provider->parseImuData();

    if(use_gt){
        std::string gt_sensor_name = "state_groundtruth_estimate0";
        data_provider->parseGtData(gt_sensor_name);
    }

    CameraImageLists left_image_list = data_provider->left_cam_image_list_;
    CameraImageLists right_image_list = data_provider->right_cam_image_list_;

    ROS_INFO_STREAM("Writing left image message to bag File: " << bag_filename);
    bag_writer->writeImageMsg("cam0", cam0_topic, left_image_list);
    ROS_INFO_STREAM("Writing right image message to bag File: " << bag_filename);
    bag_writer->writeImageMsg("cam1", cam1_topic, right_image_list);

    std::vector<ImuMeasurement> imu_measurements = data_provider->imu_measurements_;
    ROS_INFO_STREAM("Writing IMU message to bag File: " << bag_filename);
    bag_writer->writeImuMsg("imu0", imu_measurements);

    ROS_INFO_STREAM("Writing pose message to bag File: " << bag_filename);
    bag_writer->writeGTPose("/svin2/transform_stamped", data_provider->ground_truth_,
                            left_image_list.img_lists_);


}