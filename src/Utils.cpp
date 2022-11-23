#include "Utils.h"

#include <ros/console.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <signal.h>

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

    const cv::Mat readRosImage(const sensor_msgs::ImageConstPtr &img_msg, bool grayscale)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg);
        }
        catch (cv_bridge::Exception &exception)
        {
            ROS_FATAL("cv_bridge exception: %s", exception.what());
            raise(SIGABRT);
        }

        const cv::Mat img_const = cv_ptr->image; // Don't modify shared image in ROS.
        cv::Mat converted_img;
        if (cv_ptr->encoding == sensor_msgs::image_encodings::BGR8)
        {
            if (grayscale)
            {
                cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
            }
            return converted_img;
        }
        else if (cv_ptr->encoding == sensor_msgs::image_encodings::RGB8)
        {
            if (grayscale)
            {
                cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
            }
            else
            {
                cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2BGR);
            }
            return converted_img;
        }

        return img_const;
    }

    const cv::Mat readCompressedRosImage(const sensor_msgs::CompressedImageConstPtr &img_msg, bool grayscale)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg);
        }
        catch (cv_bridge::Exception &exception)
        {
            ROS_FATAL("cv_bridge exception: %s", exception.what());
            raise(SIGABRT);
        }

        const cv::Mat img_const = cv_ptr->image; // Don't modify shared image in ROS.
        cv::Mat converted_img;
        if (cv_ptr->encoding == sensor_msgs::image_encodings::BGR8)
        {
            if (grayscale)
            {
                cv::cvtColor(img_const, converted_img, cv::COLOR_BGR2GRAY);
            }
            return img_const;
        }
        else if (cv_ptr->encoding == sensor_msgs::image_encodings::RGB8)
        {
            if (grayscale)
            {
                cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2GRAY);
            }
            else
            {
                cv::cvtColor(img_const, converted_img, cv::COLOR_RGB2BGR);
            }
            return converted_img;
        }

        return img_const;
    }

    void readIntrinsics(cv::FileNode &cam_node, cv::Mat &K, cv::Mat &distortion_coeffs)
    {
        cv::FileNode knode = cam_node["K"]["data"];
        if (knode.isSeq())
        {
            K.at<double>(0, 0) = static_cast<double>(knode[0]);
            K.at<double>(1, 1) = static_cast<double>(knode[4]);
            K.at<double>(0, 2) = static_cast<double>(knode[2]);
            K.at<double>(1, 2) = static_cast<double>(knode[5]);
        }
        else
        {
            ROS_ERROR_STREAM("Camera Matrix is not a sequence");
        }

        cv::FileNode dnode = cam_node["D"]["data"];

        if (dnode.isSeq())
        {
            distortion_coeffs.at<double>(0, 0) = static_cast<double>(dnode[0]);
            distortion_coeffs.at<double>(1, 0) = static_cast<double>(dnode[1]);
            distortion_coeffs.at<double>(2, 0) = static_cast<double>(dnode[2]);
            distortion_coeffs.at<double>(3, 0) = static_cast<double>(dnode[3]);
        }
        else
        {
            ROS_ERROR_STREAM("Distortion coeffs is not a sequence");
        }
    }
}
