#include "Utils.h"
#include "PoseInterpolator.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <fstream>
#include <iostream>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "interpolator");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    std::string ground_truth_file, trajectory_file, result_file;
    ROS_FATAL_STREAM_COND(!nh_private.getParam("gt_file", ground_truth_file),
                          "Ground truth (COLMAP) file path not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("trajectory_file", trajectory_file),
                          "VIO trajectory file not set from params");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("result_file", result_file),
                          "Result file not set from params");
    bool skip_first_line = false;
    nh_private.param("skip_first_line", skip_first_line, skip_first_line);

    if (result_file.empty() || ground_truth_file.empty() || trajectory_file.empty())
        exit(1);

    std::vector<std::uint64_t> camera_timestamps;
    Utils::getStampsFromTrajectory(ground_truth_file, camera_timestamps, skip_first_line);
    ROS_INFO_STREAM("Got " << camera_timestamps.size() << " images\n");

    std::unique_ptr<PoseInterpolator> pose_intr = std::make_unique<PoseInterpolator>(trajectory_file);
    pose_intr->loadTrajectory(' ', skip_first_line);

    geometry_msgs::Pose pose;
    std::ofstream outfile(result_file);

    for (int i = 0; i < camera_timestamps.size(); ++i)
    {
        std::uint64_t stamp = camera_timestamps.at(i);
        if (pose_intr->getPose(stamp, pose))
        {
            // ROS_INFO_STREAM("Pose: " << pose << std::endl);
            outfile << std::setprecision(16) << ((double)stamp) / 1000000000 << " ";
            outfile << std::setprecision(6) << pose.position.x << " " << pose.position.y << " "
                    << pose.position.z << " " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z
                    << " " << pose.orientation.w << std::endl;
        }
        else
        {
            ROS_WARN_STREAM(std::setprecision(16) << "Finding pose at: " << ((double)stamp) / 1000000000);
            ROS_WARN_STREAM("Pose not found\n");
        }
    }
    outfile.close();

    return 0;
}