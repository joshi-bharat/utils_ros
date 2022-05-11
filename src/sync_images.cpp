#include "StereoSyncOnline.h"

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sync_images_online");

    std::unique_ptr<StereoSyncOnline> stereo_sync = std::make_unique<StereoSyncOnline>();

    while (ros::ok())
    {
        ros::spinOnce();
    }
}