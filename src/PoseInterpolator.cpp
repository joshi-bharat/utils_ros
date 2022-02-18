#include "PoseInterpolator.h"

#include <fstream>
#include <ros/console.h>

#include <Eigen/Core>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PoseInterpolator::PoseInterpolator(std::string &traj_file)
    : traj_file_(traj_file)
{
    ROS_FATAL_STREAM_COND(traj_file_.empty(), "Trajectory file " << traj_file_ << " is empty.");
}

bool PoseInterpolator::loadTrajectory(char separator, bool skip_header)
{
    ROS_INFO_STREAM("Parsing VIO trajectory data ....");
    std::ifstream fin(traj_file_.c_str());

    ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << traj_file_ << '\n');

    // Skip the first line, containing the header.
    std::string line;
    if (skip_header)
        std::getline(fin, line);

    while (std::getline(fin, line))
    {
        std::uint64_t timestamp = 0;
        std::vector<double> data_raw;
        for (size_t i = 0u; i < 9; i++)
        {
            int idx = line.find_first_of(separator);
            if (i == 0u)
            {
                timestamp = (std::uint64_t)(std::stold(line.substr(0, idx)) * 1000000000);
            }
            else
            {
                data_raw.push_back(std::stod(line.substr(0, idx)));
            }
            line = line.substr(idx + 1);
        }

        geometry_msgs::Point position;
        position.x = data_raw[0];
        position.y = data_raw[1];
        position.z = data_raw[2];

        // Quaternion x y z w.

        // Sanity check.
        Eigen::Vector4d q(data_raw[3], data_raw[4], data_raw[5], data_raw[6]);

        geometry_msgs::Quaternion rot;
        rot.x = q(0);
        rot.y = q(1);
        rot.z = q(2);
        rot.w = q(3);

        geometry_msgs::Pose pose;
        pose.position = position;
        pose.orientation = rot;

        // ROS_INFO_STREAM("Inserting pose at stamp: " << timestamp << std::endl
        // << pose << std::endl);
        trajectory_.insert(std::make_pair(timestamp, pose));
    }

    ROS_INFO_STREAM("Added " << trajectory_.size() << " poses to trajectory.\n");

    return true;
}

bool PoseInterpolator::getPose(const std::uint64_t &timestamp, geometry_msgs::Pose &pose)
{
    auto itr_low = trajectory_.lower_bound(timestamp);
    auto itr_upp = trajectory_.upper_bound(timestamp);

    if (itr_upp == trajectory_.begin())
    {
        ROS_WARN_STREAM("Provided timestamp is too early. Trajectory only starts at " << trajectory_.begin()->first);
        return false;
    }
    else if (itr_low == trajectory_.end() && itr_low->first < timestamp)
    {
        ROS_WARN_STREAM("Provided timestamp is too large. Trajectory ends at " << (--trajectory_.end())->first);
        return false;
    }
    else if (itr_low->first == timestamp)
    {
        // Exactly matches
        pose.position = itr_low->second.position;
        pose.orientation = itr_low->second.orientation;
        return true;
    }
    else
    {
        // ROS_INFO(" !!!!!!!! here !!!!!!!");
        std::uint64_t upper = itr_low->first;
        geometry_msgs::Pose upper_pose = itr_low->second;
        geometry_msgs::Point pos = upper_pose.position;
        Eigen::Vector3d trans_upper(pos.x, pos.y, pos.z);
        tf2::Quaternion tf2_quat_up;
        tf2::convert(upper_pose.orientation, tf2_quat_up);

        std::uint64_t lower = (--itr_low)->first;
        geometry_msgs::Pose lower_pose = itr_low->second;
        pos = lower_pose.position;
        Eigen::Vector3d trans_lower(pos.x, pos.y, pos.z);
        tf2::Quaternion tf2_quat_low;
        tf2::convert(lower_pose.orientation, tf2_quat_low);

        double ratio = ((double)(timestamp - lower)) / ((double)(upper - lower));
        tf2::Quaternion quat = tf2_quat_low.slerp(tf2_quat_up, ratio);
        Eigen::Vector3d trans = ratio * trans_upper + (1.0 - ratio) * trans_lower;

        pose.position.x = trans.x();
        pose.position.y = trans.y();
        pose.position.z = trans.z();

        tf2::convert(quat, pose.orientation);
        return true;
    }
}