
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <imagenex831l/ProcessedRange.h>

rosbag::Bag bag;
std::string sonar_topic = "/sonar0";
bool bag_opened = false;

void callbackSonar(const imagenex831l::ProcessedRange::ConstPtr &sonar_msg)
{
    ROS_WARN_ONCE("!!! Inside Sonar Callback !!!");
    if (bag_opened)
    {
        bag.write(sonar_topic, sonar_msg->header.stamp, sonar_msg);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "add_sonar");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
    std::string rosbag_filename;

    ROS_FATAL_STREAM_COND(!nh_private.getParam("rosbag_filename", rosbag_filename),
                          "RosBag file param not found");

    ros::Subscriber sub = nh.subscribe("/imagenex831l/range", 100u, &callbackSonar);

    if (!bag_opened)
    {
        bag.open(rosbag_filename, rosbag::bagmode::Append);
        bag_opened = true;
    }

    while (ros::ok() && !ros::Time::now().isValid())
    {
        if (ros::Time::isSimTime())
        {
            ROS_INFO_STREAM_ONCE("Waiting for ROS time to be valid... \n"
                                 << "(Sim Time is enabled; run rosbag with --clock argument)");
        }
        else
        {
            ROS_INFO_STREAM_ONCE("Waiting for ROS time to be valid...");
        }
    }

    while (ros::ok())
    {
        ros::spinOnce();
    }

    bag.close();
    return 0;
}