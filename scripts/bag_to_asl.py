#!/usr/bin/python

import os

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from tqdm import tqdm


def extract_bag(
    dataset_folder, bagfile, left_image_topic, right_image_topic, imu_topic
):

    bag = rosbag.Bag(bagfile, "r")
    left_image_folder = os.path.join(dataset_folder, "cam0")
    right_image_folder = os.path.join(dataset_folder, "cam1")
    imu_folder = os.path.join(dataset_folder, "imu0")

    if not os.path.exists(left_image_folder):
        os.makedirs(left_image_folder)
    if not os.path.exists(right_image_folder):
        os.makedirs(right_image_folder)
    if not os.path.exists(imu_folder):
        os.makedirs(imu_folder)

    left_image_file = open(os.path.join(left_image_folder, "data.csv"), "w")
    right_image_file = open(os.path.join(right_image_folder, "data.csv"), "w")
    imu_file = open(os.path.join(imu_folder, "data.csv"), "w")

    left_image_data_folder = os.path.join(left_image_folder, "data")
    right_image_data_folder = os.path.join(right_image_folder, "data")
    if not os.path.exists(left_image_data_folder):
        os.makedirs(left_image_data_folder)
    if not os.path.exists(right_image_data_folder):
        os.makedirs(right_image_data_folder)

    topics = [left_image_topic, right_image_topic, imu_topic]

    cv_bridge = CvBridge()

    imu_file.write(
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
    )
    imu_file.write("\n")
    left_image_file.write("#timestamp [ns],filename\n")
    right_image_file.write("#timestamp [ns],filename\n")

    orb = open(os.path.join(dataset_folder, "timestamps.txt"), "w")
    for topic, msg, t in tqdm(bag.read_messages(), total=bag.get_message_count()):  # type: ignore
        if topic in topics:
            stamp = str(msg.header.stamp.to_nsec())
            if msg._type == "sensor_msgs/Image" and topic == left_image_topic:
                cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                filename = os.path.join(left_image_data_folder, stamp + ".png")
                cv2.imwrite(filename, cv_image)
                left_image_file.write(stamp + "," + stamp + ".png" + "\n")
                orb.write(stamp + "\n")
            elif (
                msg._type == "sensor_msgs/CompressedImage" and topic == left_image_topic
            ):
                filename = os.path.join(left_image_data_folder, stamp + ".png")
                cv_image = cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(filename, cv_image)
                left_image_file.write(stamp + "," + stamp + ".png" + "\n")
                orb.write(stamp + "\n")
            elif msg._type == "sensor_msgs/Image" and topic == right_image_topic:
                cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                filename = os.path.join(right_image_data_folder, stamp + ".png")
                cv2.imwrite(filename, cv_image)
                right_image_file.write(stamp + "," + stamp + ".png" + "\n")
            elif (
                msg._type == "sensor_msgs/CompressedImage"
                and topic == right_image_topic
            ):
                filename = os.path.join(right_image_data_folder, stamp + ".png")
                cv_image = cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(filename, cv_image)
                right_image_file.write(stamp + "," + stamp + ".png" + "\n")
            elif msg._type == "sensor_msgs/Imu":
                acc = msg.linear_acceleration
                ang_vel = msg.angular_velocity
                imu_file.write(
                    "{},{},{},{},{},{},{}\n".format(
                        stamp, ang_vel.x, ang_vel.y, ang_vel.z, acc.x, acc.y, acc.z
                    )
                )
    imu_file.close()
    left_image_file.close()
    right_image_file.close()
    bag.close()


if __name__ == "__main__":
    rospy.init_node("gopro_bag_to_asl")

    if not rospy.has_param("~dataset_path"):
        rospy.logfatal("Require the dataset path of asl directory")

    if not rospy.has_param("~bag"):
        rospy.logfatal("Require the bag file for extraction")

    if not rospy.has_param("~left_image_topic"):
        rospy.logfatal("Require the image_topic")
    if not rospy.has_param("~right_image_topic"):
        rospy.logfatal("Require the image_topic")

    if not rospy.has_param("~imu_topic"):
        rospy.logfatal("Require the imu topic")

    dataset_folder = rospy.get_param("~dataset_path")
    bagfile = rospy.get_param("~bag")
    left_image_topic = rospy.get_param("~left_image_topic")
    right_image_topic = rospy.get_param("~right_image_topic")
    imu_topic = rospy.get_param("~imu_topic")

    print(dataset_folder)
    print(bagfile)
    print(left_image_topic)
    print(right_image_topic)
    print(imu_topic)

    extract_bag(dataset_folder, bagfile, left_image_topic, right_image_topic, imu_topic)
