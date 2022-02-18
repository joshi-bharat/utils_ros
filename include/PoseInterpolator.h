#pragma once

/// Saves all the poses from visual odometry
/// Then, get pose of particular image using slerp for quaternion interpolation

#include <string>

#include <geometry_msgs/Pose.h>

class PoseInterpolator
{
private:
    /* data */
public:
    PoseInterpolator(std::string &traj_file);
    ~PoseInterpolator() = default;

    bool loadTrajectory(char separator = ' ', bool skip_header = false);
    bool getPose(const std::uint64_t &timestamp, geometry_msgs::Pose &pose);

private:
    const std::string traj_file_;
    std::map<std::uint64_t, geometry_msgs::Pose> trajectory_;
};
