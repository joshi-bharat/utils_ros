#include <fstream>
#include <ros/console.h>

#include "Definitions.h"

bool CameraImageLists::parseCamImgList(const std::string& folderpath,
                                       const std::string& filename) {
  ROS_FATAL_STREAM_COND(folderpath.empty(), "Camera Folder Empty: " << folderpath);
  ROS_FATAL_STREAM_COND(filename.empty(), "Camera file " << filename << " does not exist");

  image_folder_path_ = folderpath;  // stored, only for debug
  const std::string fullname = folderpath + "/" + filename;
  std::ifstream fin(fullname.c_str());
  ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << fullname);

  // Skip the first line, containing the header.
  std::string item;
  std::getline(fin, item);

  // Read/store list of image names.
  while (std::getline(fin, item)) {
    // Print the item!
    auto idx = item.find_first_of(',');
    Timestamp timestamp = std::stoll(item.substr(0, idx));
    std::string image_filename =
        folderpath + "/data/" + item.substr(0, idx) + ".png";
    // Strangely, on mac, it does not work if we use: item.substr(idx + 1);
    // maybe different string termination characters???
    img_lists_.push_back(make_pair(timestamp, image_filename));
  }
  fin.close();
  return true;
}

/* -------------------------------------------------------------------------- */
void CameraImageLists::print() const {
  ROS_INFO_STREAM("------------ CameraImageLists::print -------------\n"
            << "image_folder_path: " << image_folder_path_ << '\n'
            << "img_lists size: " << img_lists_.size());
}