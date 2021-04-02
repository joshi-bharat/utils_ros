#include <fstream>

#include <ros/console.h>
#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>

#include "EurocDataProvider.h"

bool interpolate_poses(const std::map<Timestamp, gtsam::Pose3> &gt_data,
                    const std::vector<std::pair<Timestamp, std::string>>& img_lists,
                    const std::string& result_path){
    
    ofstream results;
    results.open(result_path);
    results << "#timestamp px py pz qx qy qz qw\n";

    for (std::uint64_t i = 0; i < img_lists.size(); ++i){

        std::pair<Timestamp, std::string> stamp_img_pair = img_lists.at(i);
        Timestamp stamp = stamp_img_pair.first;

        gtsam::Point3 t;
        tf2::Quaternion q;

        if (stamp < gt_data.begin()->first){
            ROS_WARN_STREAM("Ground truth not available for timestamp: " 
            << stamp << ". Ground truth starts at: " << gt_data.begin()->first);
            t = gtsam::Point3(0.0, 0.0, 0.0);
            q = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
        }else{

            auto itr_low = gt_data.equal_range(stamp).first;
            if(itr_low == gt_data.begin() || stamp == itr_low->first){
                //Ground truth does not have lower timestamp or matches exactly
                const gtsam::Point3& trans = itr_low->second.translation();
                const gtsam::Quaternion &quat = itr_low->second.rotation().toQuaternion();

                t = gtsam::Point3(trans.x(), trans.y(), trans.z());
                q = tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
            
            }else{
                //Interpolate

                Timestamp upper = itr_low->first;
                gtsam::Pose3 upper_pose = itr_low->second;
                gtsam::Point3 trans_upper = upper_pose.translation();
                gtsam::Quaternion q_up = upper_pose.rotation().toQuaternion();
                tf2::Quaternion tf2_quat_up(q_up.x(), q_up.y(), q_up.z(), q_up.w());

                Timestamp lower = (--itr_low)->first;
                gtsam::Pose3 lower_pose = itr_low->second;
                gtsam::Point3 trans_lower = lower_pose.translation();
                gtsam::Quaternion ql = lower_pose.rotation().toQuaternion();
                tf2::Quaternion tf2_quat_low(ql.x(), ql.y(), ql.z(), ql.w());

                double ratio = ((double)(stamp - lower)) / ((double)(upper - lower));
                tf2::Quaternion quat = tf2_quat_low.slerp(tf2_quat_up, ratio);
                gtsam::Point3 trans = ratio * trans_upper + (1.0 - ratio) * trans_lower;


                t = gtsam::Point3(trans.x(), trans.y(), trans.z());
                q = tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());

            }
        }

        results << stamp << " " << t.x() << " " << t.y() << " " << t.z()
            << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";  
    }
    results.close();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose_interpolate");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string dataset_path;
    if (!nh_private.getParam("dataset_path", dataset_path))
        ROS_FATAL_STREAM("Dataset path not provided\n");

    std::string interpolated_file;
    if (!nh_private.getParam("interpolated_file", interpolated_file))
        ROS_FATAL_STREAM("Need filename to write the interpolated result\n");

    std::unique_ptr<EurocDataProvider> data_provider = std::make_unique<EurocDataProvider>(dataset_path);
    data_provider->parseLeftCam();
    std::string gt_sensor_name = "state_groundtruth_estimate0";
    data_provider->parseGtData(gt_sensor_name);

    CameraImageLists left_image_list = data_provider->left_cam_image_list_;

    interpolate_poses(data_provider->ground_truth_, left_image_list.img_lists_, interpolated_file);

}