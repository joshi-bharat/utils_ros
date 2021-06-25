#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include "BagWriter.h"
#include "Utils.h"

BagWriter::BagWriter(const std::string &bag_file, rosbag::BagMode mode)
{
    bag_filename_ = bag_file;
    ROS_FATAL_STREAM_COND(mode == rosbag::bagmode::Read, "BagWrite only supports append and write mode");
    bag_.open(bag_filename_, mode);
}

BagWriter::~BagWriter()
{
    ROS_INFO("!! Bag Writer Destructor called. !!");
    bag_.close();
}

bool BagWriter::writeImageMsg(const std::string &frame_id, const std::string &topic_name, const CameraImageLists &img_list)
{

    using namespace std;

    for (uint64_t frame = 0; frame < img_list.img_lists_.size(); ++frame)
    {
        Timestamp stamp = img_list.img_lists_[frame].first;
        std::string image_filename = img_list.img_lists_[frame].second;

        // cout << "Frame: " << frame << "\tStamp: " << stamp << "\tFilename: " << image_filename << endl;
        cv::Mat image = cv::imread(image_filename);
        bool is_colored = false;
        if (image.channels() == 3)
        {
            is_colored = true;
        }

        uint32_t secs = stamp * 1e-9;
        uint32_t n_secs = stamp % 1000000000;
        ros::Time ros_time(secs, n_secs);
        // cout <<  "Stamp: " << stamp << "\tSecs: " << secs << "\tnsecs: " << n_secs << endl;
        // break;
        std_msgs::Header header;
        header.seq = frame;
        header.frame_id = frame_id;
        header.stamp = ros_time;
        sensor_msgs::ImagePtr msg;
        if (is_colored)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
            msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image).toImageMsg();
        }
        else
        {
            msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image).toImageMsg();
        }

        ROS_INFO_STREAM_ONCE("Image Width : " << msg->width << "\tHeight : " << msg->height << "\tEncoding : " << msg->encoding);

        bag_.write(topic_name, ros_time, msg);
    }
    return true;
}

bool BagWriter::writeImuMsg(const std::string &imu_topic, const std::vector<ImuMeasurement> &imu_measurements)
{

    std::int64_t previous_timestamp = -1;
    for (std::uint64_t i = 0; i < imu_measurements.size(); ++i)
    {

        ImuMeasurement imu_meas = imu_measurements.at(i);

        ROS_FATAL_STREAM_COND((std::int64_t)imu_meas.timestamp_ <= previous_timestamp,
                              "IMU data is not in chronological order!");
        previous_timestamp = (std::int64_t)imu_meas.timestamp_;

        Timestamp stamp = imu_meas.timestamp_;
        std::uint32_t secs = stamp * 1e-9;
        std::uint32_t n_secs = stamp % 1000000000;
        ros::Time ros_time(secs, n_secs);

        std_msgs::Header header;
        header.seq = i;
        header.frame_id = "imu0";
        header.stamp = ros_time;

        sensor_msgs::Imu imu_msg;
        imu_msg.header = header;
        imu_msg.linear_acceleration.x = imu_meas.acc_gyr_(0, 0);
        imu_msg.linear_acceleration.y = imu_meas.acc_gyr_(1, 0);
        imu_msg.linear_acceleration.z = imu_meas.acc_gyr_(2, 0);
        imu_msg.angular_velocity.x = imu_meas.acc_gyr_(3, 0);
        imu_msg.angular_velocity.y = imu_meas.acc_gyr_(4, 0);
        imu_msg.angular_velocity.z = imu_meas.acc_gyr_(5, 0);

        bag_.write(imu_topic, ros_time, imu_msg);
    }
}

bool BagWriter::writeGTPose(const std::string &gt_topic,
                            const std::map<Timestamp, gtsam::Pose3> &gt_data,
                            const std::vector<std::pair<Timestamp, std::string>> &img_lists,
                            bool write_nav_path)
{

    std::vector<geometry_msgs::PoseStamped> gt_poses;

    for (std::uint64_t i = 0; i < img_lists.size(); ++i)
    {

        std::pair<Timestamp, std::string> stamp_img_pair = img_lists.at(i);
        Timestamp stamp = stamp_img_pair.first;

        std::uint32_t secs = stamp * 1e-9;
        std::uint32_t n_secs = stamp % 1000000000;
        ros::Time ros_time(secs, n_secs);

        std_msgs::Header header;
        header.seq = i;
        header.frame_id = "/world";
        header.stamp = ros_time;

        geometry_msgs::TransformStamped transform_stamped;
        geometry_msgs::Transform transform;

        transform_stamped.header = header;
        transform_stamped.child_frame_id = "/svin2_pose";

        if (stamp < gt_data.begin()->first)
        {
            ROS_WARN_STREAM("Ground truth not available for timestamp: "
                            << stamp << ". Ground truth starts at: " << gt_data.begin()->first);
            transform.translation.x = 0.0;
            transform.translation.y = 0.0;
            transform.translation.z = 0.0;
            transform.rotation.x = 0.0;
            transform.rotation.y = 0.0;
            transform.rotation.z = 0.0;
            transform.rotation.w = 1.0;
        }
        else
        {

            auto itr_low = gt_data.equal_range(stamp).first;
            if (itr_low == gt_data.begin() || stamp == itr_low->first)
            {
                //Ground truth does not have lower timestamp or matches exactly
                const gtsam::Point3 &trans = itr_low->second.translation();
                const gtsam::Quaternion &quat = itr_low->second.rotation().toQuaternion();

                transform.translation.x = trans.x();
                transform.translation.y = trans.y();
                transform.translation.z = trans.z();
                transform.rotation.x = quat.x();
                transform.rotation.y = quat.y();
                transform.rotation.z = quat.z();
                transform.rotation.w = quat.w();
            }
            else
            {
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

                transform.translation.x = trans.x();
                transform.translation.y = trans.y();
                transform.translation.z = trans.z();
                transform.rotation.x = quat.x();
                transform.rotation.y = quat.y();
                transform.rotation.z = quat.z();
                transform.rotation.w = quat.w();

                // ROS_WARN_STREAM("Calculating pose at stamp: " << stamp << " between stamps: " << lower << " and " << upper);

                // utils::printTf2Quat(tf2_quat_low, "Lower: ");
                // utils::printTf2Quat(tf2_quat_up, "Upper: ");
                // std::cout << "Ratio: " << ratio << "\n";
                // utils::printTf2Quat(quat, "Result: ");

                // std::cout << "Lower: " << trans_lower << "\n" <<
                //   "Upper: " << trans_upper << "\n" <<
                //   "Ratio: " << ratio << "\n"
                //   << "Result: " << trans << "\n";
                // if (i > 10)
                // break;
            }

            // ROS_INFO_STREAM("Got GT pose at low: " <<)
        }

        transform_stamped.transform = transform;
        bag_.write(gt_topic, ros_time, transform_stamped);

        // Since RVIZ does not have viz for transform stamped, let's use pose stamped

        if (write_nav_path)
        {
            geometry_msgs::Pose pose;
            pose.position.x = transform.translation.x;
            pose.position.y = transform.translation.y;
            pose.position.z = transform.translation.z;
            pose.orientation.x = transform.rotation.x;
            pose.orientation.y = transform.rotation.y;
            pose.orientation.z = transform.rotation.z;
            pose.orientation.w = transform.rotation.w;

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.pose = pose;

            gt_poses.emplace_back(pose_stamped);

            nav_msgs::Path path_msg;
            path_msg.header = header;
            path_msg.poses = gt_poses;

            bag_.write("/svin2/gt_path", ros_time, path_msg);
        }
    }
}

bool BagWriter::writeGTPose(const std::string &gt_topic,
                            const std::map<Timestamp, gtsam::Pose3> &gt_data,
                            const std::vector<Timestamp> &img_stamps,
                            bool write_nav_path)
{
    std::vector<geometry_msgs::PoseStamped> gt_poses;
    for (std::uint64_t i = 0; i < img_stamps.size(); ++i)
    {
        Timestamp stamp = img_stamps.at(i);

        std::uint32_t secs = stamp * 1e-9;
        std::uint32_t n_secs = stamp % 1000000000;
        ros::Time ros_time(secs, n_secs);

        std_msgs::Header header;
        header.seq = i;
        header.frame_id = "world";
        header.stamp = ros_time;

        geometry_msgs::TransformStamped transform_stamped;
        geometry_msgs::Transform transform;

        transform_stamped.header = header;
        transform_stamped.child_frame_id = "svin2_pose";

        if (stamp < gt_data.begin()->first)
        {
            ROS_WARN_STREAM("Ground truth not available for timestamp: "
                            << stamp << ". Ground truth starts at: " << gt_data.begin()->first);
            transform.translation.x = 0.0;
            transform.translation.y = 0.0;
            transform.translation.z = 0.0;
            transform.rotation.x = 0.0;
            transform.rotation.y = 0.0;
            transform.rotation.z = 0.0;
            transform.rotation.w = 1.0;
        }
        else if (stamp <= std::prev(gt_data.end())->first)
        {

            auto itr_low = gt_data.equal_range(stamp).first;

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

            transform.translation.x = trans.x();
            transform.translation.y = trans.y();
            transform.translation.z = trans.z();
            transform.rotation.x = quat.x();
            transform.rotation.y = quat.y();
            transform.rotation.z = quat.z();
            transform.rotation.w = quat.w();

            // ROS_WARN_STREAM("Calculating pose at stamp: " << stamp << " between stamps: " << lower << " and " << upper);

            // utils::printTf2Quat(tf2_quat_low, "Lower: ");
            // utils::printTf2Quat(tf2_quat_up, "Upper: ");
            // std::cout << "Ratio: " << ratio << "\n";
            // utils::printTf2Quat(quat, "Result: ");

            // std::cout << "Lower: " << trans_lower << "\n" <<
            //   "Upper: " << trans_upper << "\n" <<
            //   "Ratio: " << ratio << "\n"
            //   << "Result: " << trans << "\n";
            // if (i > 10)
            // break;

            // ROS_INFO_STREAM("Got GT pose at low: " <<)
        }
        else if (stamp >= std::prev(gt_data.end())->first)
        {
            ROS_WARN_STREAM("Ground truth not available for timestamp: "
                            << stamp << ". Ground truth ends at: " << std::prev(gt_data.end())->first);
            gtsam::Pose3 last_pose = std::prev(gt_data.end())->second;
            gtsam::Point3 trans = last_pose.translation();
            gtsam::Quaternion quat = last_pose.rotation().toQuaternion();

            transform.translation.x = trans.x();
            transform.translation.y = trans.y();
            transform.translation.z = trans.z();
            transform.rotation.x = quat.x();
            transform.rotation.y = quat.y();
            transform.rotation.z = quat.z();
            transform.rotation.w = quat.w();
        }

        transform_stamped.transform = transform;
        bag_.write(gt_topic, ros_time, transform_stamped);

        // Since RVIZ does not have viz for transform stamped, let's use pose stamped

        if (write_nav_path)
        {
            geometry_msgs::Pose pose;
            pose.position.x = transform.translation.x;
            pose.position.y = transform.translation.y;
            pose.position.z = transform.translation.z;
            pose.orientation.x = transform.rotation.x;
            pose.orientation.y = transform.rotation.y;
            pose.orientation.z = transform.rotation.z;
            pose.orientation.w = transform.rotation.w;

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = header;
            pose_stamped.pose = pose;

            gt_poses.emplace_back(pose_stamped);

            nav_msgs::Path path_msg;
            path_msg.header = header;
            path_msg.poses = gt_poses;

            bag_.write("/svin2/gt_path", ros_time, path_msg);
        }
    }
}