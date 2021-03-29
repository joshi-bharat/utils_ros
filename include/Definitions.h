#pragma once

#include <vector>
#include <Eigen/Core>

using Timestamp = std::uint64_t;
using ImuStamp = std::uint64_t;
using ImuAccGyr = Eigen::Matrix<double, 6, 1>;

struct ImuMeasurement
{
    ImuMeasurement() = default;
    ImuMeasurement(const ImuStamp &timestamp, const ImuAccGyr &imu_data)
        : timestamp_(timestamp), acc_gyr_(imu_data) {}
    ImuMeasurement(ImuStamp &&timestamp, ImuAccGyr &&imu_data)
        : timestamp_(std::move(timestamp)), acc_gyr_(std::move(imu_data)) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuStamp timestamp_;
    ImuAccGyr acc_gyr_;
};

 /* Store a list of image names and provide functionalities to parse them.
 */
class CameraImageLists {
 public:
  bool parseCamImgList(const std::string& folderpath,
                       const std::string& filename);
  inline size_t getNumImages() const { return img_lists_.size(); }
  void print() const;

 public:
  std::string image_folder_path_;
  typedef std::pair<Timestamp, std::string> TimestampToFilename;
  typedef std::vector<TimestampToFilename> ImgLists;
  ImgLists img_lists_;
};
