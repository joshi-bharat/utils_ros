#include "Utils.h"

#include <ros/console.h>
#include <fstream>

namespace Utils
{

    bool getImageStamps(const std::string &folder_path,
                        std::vector<std::uint64_t> &time_stamps)
    {

        ROS_FATAL_STREAM_COND(folder_path.empty(), "Camera Folder Empty: " << folder_path);
        const std::string stamp_file = folder_path + "/" + "data.csv";

        std::ifstream fin(stamp_file.c_str());
        ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << stamp_file);

        // Skip the first line, containing the header.
        std::string item;
        std::getline(fin, item);

        // Read/store list of image names.
        while (std::getline(fin, item))
        {
            // Print the item!
            auto idx = item.find_first_of(',');
            std::uint64_t timestamp = std::stoll(item.substr(0, idx));
            time_stamps.push_back(timestamp);
        }

        fin.close();

        return true;
    }

    bool getStampsFromTrajectory(const std::string &trajectory_file,
                                 std::vector<std::uint64_t> &time_stamps,
                                 bool skip_first_line)
    {

        std::ifstream fin(trajectory_file.c_str());
        ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << trajectory_file);

        // Skip the first line, containing the header.
        std::string item;
        if (skip_first_line)
            std::getline(fin, item);

        // Read/store list of image names.
        while (std::getline(fin, item))
        {
            // Print the item!
            auto idx = item.find_first_of(' ');
            std::uint64_t timestamp = (std::uint64_t)(std::stold(item.substr(0, idx)) * 1000000000);
            time_stamps.push_back(timestamp);
        }

        fin.close();

        return true;
    }
}