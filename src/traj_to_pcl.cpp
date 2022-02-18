#include <fstream>
#include <string>

#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

void getTrajectory(const std::string &file, std::vector<Eigen::Vector3d> &points,
                   char separator = ' ', bool skip_firstline = true)
{
    std::ifstream fin(file.c_str());

    ROS_FATAL_STREAM_COND(!fin.is_open(), "Cannot open file: " << file << '\n');

    // Skip the first line, containing the header.
    std::string line;
    if (skip_firstline)
        std::getline(fin, line);

    while (std::getline(fin, line))
    {
        double timestamp = 0;
        std::vector<double> data_raw;
        for (size_t i = 0u; i < 9; i++)
        {
            int idx = line.find_first_of(separator);
            if (i == 0u)
            {
                timestamp = std::stold(line.substr(0, idx));
            }
            else
            {
                data_raw.push_back(std::stod(line.substr(0, idx)));
            }
            line = line.substr(idx + 1);
        }

        Eigen::Vector3d trans(data_raw[0], data_raw[1], data_raw[2]);

        points.emplace_back(trans);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "correct_traj");
    ros::NodeHandle nh;

    std::string pkg_path = ros::package::getPath("iccv_utils");

    std::string file = pkg_path + "/traj/svin_test.txt";
    std::string ply_file = pkg_path + "/svin_traj.ply";

    std::vector<Eigen::Vector3d> points;
    getTrajectory(file, points, ' ', true);
    pcl::PLYWriter writer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const Eigen::Vector3d &p : points)
    {
        pcl::PointXYZRGB pt;
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloud->push_back(pt);
    }

    writer.write(ply_file, *cloud);
}