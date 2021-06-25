#include <BagSync.h>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sync_images");

    std::unique_ptr<BagSync> bag_sync = std::make_unique<BagSync>();

    while (ros::ok())
    {
        ros::spinOnce();
    }
}