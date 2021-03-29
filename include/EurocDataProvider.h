#pragma once

#include <string>
#include <gtsam/geometry/Pose3.h>

#include "Definitions.h"

class EurocDataProvider
{
private:
    std::string dataset_path_;
    std::uint64_t frame_id_;
    
    const std::string kLeftCamName = "cam0";
    const std::string kRightCamName = "cam1";
    const std::string kImuName = "imu0";


public:
    EurocDataProvider(const std::string & dataset_path);
    ~EurocDataProvider();

public:

    std::vector<ImuMeasurement> imu_measurements_;
    CameraImageLists left_cam_image_list_;
    CameraImageLists right_cam_image_list_;
    std::map<Timestamp, gtsam::Pose3> ground_truth_;

    void parse();

    bool parseImuData();
    bool parseImuData(const std::string &input_dataset_path,
                      const std::string &imuName);

    bool parseLeftCam();
    bool parseRightCam();
    bool parseCameraData(const std::string &cam_name,
                         CameraImageLists *cam_list_i);

    bool parseGtData(const std::string& gt_sensor_name);
};
