#include <ros/console.h>
#include <fstream>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include "Definitions.h"
#include "EurocDataProvider.h"

EurocDataProvider::EurocDataProvider(const std::string &dataset_path)
    : dataset_path_(dataset_path)
{
    frame_id_ = 0;
    ROS_INFO("Parsing Euroc Dataset ... ");
}

EurocDataProvider::~EurocDataProvider()
{
    ROS_INFO("ETHDatasetParser destructor called.");
}

void EurocDataProvider::parse()
{
    ROS_INFO_STREAM("Using dataset path: " << dataset_path_);
    ROS_INFO_STREAM("Parsing IMU Data\n");

    //Parse IMU Data
    bool imu_parsed = parseImuData(dataset_path_, kImuName);

    //Parse Camera Data
    parseLeftCam();
    parseRightCam();
}

bool EurocDataProvider::parseImuData()
{
    return parseImuData(dataset_path_, kImuName);
}

bool EurocDataProvider::parseImuData(const std::string &input_dataset_path,
                                     const std::string &imuName)
{
    std::string filename_data = input_dataset_path + "/mav0/" + imuName + "/data.csv";
    std::ifstream fin(filename_data.c_str());

    ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << filename_data);
    // Skip the first line, containing the header.
    std::string line;
    std::getline(fin, line);
    std::int64_t previous_timestamp = -1;

    // Read/store imu measurements, line by line.
    while (std::getline(fin, line))
    {
        std::int64_t timestamp = 0;
        gtsam::Vector6 gyr_acc_data;
        for (size_t i = 0u; i < gyr_acc_data.size() + 1u; i++)
        {
            int idx = line.find_first_of(',');
            if (i == 0)
            {
                timestamp = std::stoll(line.substr(0, idx));
            }
            else
            {
                gyr_acc_data(i - 1) = std::stod(line.substr(0, idx));
            }
            line = line.substr(idx + 1);
        }

        ROS_FATAL_STREAM_COND(timestamp <= previous_timestamp, "Euroc IMU data is not in chronological order!"
                                                                   << "TS: " << timestamp << " PTS: " << previous_timestamp);
        gtsam::Vector6 imu_accgyr;
        // Acceleration first!
        imu_accgyr << gyr_acc_data.tail(3), gyr_acc_data.head(3);

        //! Store imu measurements
        imu_measurements_.push_back(ImuMeasurement(timestamp, imu_accgyr));
        previous_timestamp = timestamp;
    }
    fin.close();
    return true;
}

bool EurocDataProvider::parseLeftCam()
{
    ROS_INFO_STREAM("Parsing Left Camera Data\n");
    parseCameraData(kLeftCamName, &left_cam_image_list_);
    left_cam_image_list_.print();
}

bool EurocDataProvider::parseRightCam()
{
    ROS_INFO_STREAM("Parsing Right Camera Data\n");
    parseCameraData(kRightCamName, &right_cam_image_list_);
    right_cam_image_list_.print();
}

bool EurocDataProvider::parseCameraData(const std::string &cam_name,
                                        CameraImageLists *cam_list_i)
{
    cam_list_i->parseCamImgList(dataset_path_ + "/mav0/" + cam_name, "data.csv");
    return true;
}

bool EurocDataProvider::parseGtData(const std::string &gt_sensor_name, char separator, bool whole_dataset)
{

    ROS_INFO_STREAM("Parsing ground truth data ....");
    ROS_FATAL_STREAM_COND(gt_sensor_name.empty(), "!! GT sensor name is empty !!");

    std::string filename_data;

    if (!whole_dataset)
        filename_data = gt_sensor_name;
    else
        filename_data = dataset_path_ + "/mav0/" + gt_sensor_name + "/data.csv";

    std::ifstream fin(filename_data.c_str());

    ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << filename_data << '\n'
                                                               << "Assuming dataset has no ground truth...");

    // Skip the first line, containing the header.
    std::string line;
    std::getline(fin, line);

    while (std::getline(fin, line))
    {
        Timestamp timestamp = 0;
        std::vector<double> gt_data_raw;
        for (size_t i = 0u; i < 9; i++)
        {
            int idx = line.find_first_of(separator);
            if (i == 0u)
            {
                timestamp = (std::uint64_t)(std::stold(line.substr(0, idx)) * 1000000000);
            }
            else
            {
                gt_data_raw.push_back(std::stod(line.substr(0, idx)));
            }
            line = line.substr(idx + 1);
        }

        gtsam::Point3 position(gt_data_raw[0], gt_data_raw[1], gt_data_raw[2]);
        // Quaternion w x y z.
        gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
            gt_data_raw[6], gt_data_raw[3], gt_data_raw[4], gt_data_raw[5]);

        // Sanity check.
        gtsam::Vector q = rot.quaternion();
        // Figure out sign for quaternion.
        if (std::fabs(q(0) + gt_data_raw[6]) < std::fabs(q(0) - gt_data_raw[6]))
        {
            q = -q;
        }

        ROS_FATAL_STREAM_COND(
            (fabs(q(0) - gt_data_raw[6]) > 1e-3) ||
                (fabs(q(1) - gt_data_raw[3]) > 1e-3) ||
                (fabs(q(2) - gt_data_raw[4]) > 1e-3) ||
                (fabs(q(3) - gt_data_raw[5]) > 1e-3),
            "parseGTdata: wrong quaternion conversion"
                << "(" << q(0) << "," << gt_data_raw[6] << ") "
                << "(" << q(1) << "," << gt_data_raw[3] << ") "
                << "(" << q(2) << "," << gt_data_raw[4] << ") "
                << "(" << q(3) << "," << gt_data_raw[5] << ").");

        gtsam::Pose3 pose(rot, position);

        ground_truth_.insert(std::make_pair(timestamp, pose));
    }
}