#pragma once

#include <rosbag/bag.h>

#include <gtsam/geometry/Pose3.h>

#include "Definitions.h"

class BagWriter{

    private:
        std::string bag_filename_;
        rosbag::Bag bag_;

    public:
        BagWriter(const std::string& rosbag_filename);
        ~BagWriter();

    public:
        bool writeImageMsg(const std::string &frame_id, 
        const std::string& topic_name, const CameraImageLists &img_list);

        bool writeImuMsg(const std::string& imu_topic, const std::vector<ImuMeasurement> &imu_measurements);

        bool writeGTPose(const std::string &gt_topic,
                        const std::map<Timestamp, gtsam::Pose3> &gt_data,
                        const std::vector<std::pair<Timestamp, std::string>>& img_lists);
};