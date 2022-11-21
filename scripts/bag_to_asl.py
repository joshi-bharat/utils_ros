#!/usr/bin/python

import os

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from tqdm import tqdm
import numpy as np

np.set_printoptions(suppress=True)


def read_camera_intrinsics(cv_node: cv2.FileNode):
    # Read camera intrinsics from cv_node

    camera_matrix = np.ones((3, 3))
    distortion_coefficients = np.ones((1, 4))

    camera_matrix = cv_node.getNode("K").mat()
    distortion_coefficients = cv_node.getNode("D").mat()

    print("Camera Matrix: ", camera_matrix)
    print("Distortion Coefficients: ", distortion_coefficients)

    return camera_matrix, distortion_coefficients


def extract_bag(
    dataset_folder: str,
    bagfile: str,
    left_image_topic: str,
    right_image_topic: str,
    imu_topic: str,
    config_file: str,
    scale: float = 1.0,
    rectify: bool = False,
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

    cv_bridge = CvBridge()

    opencv_config = cv2.FileStorage(config_file, cv2.FILE_STORAGE_READ)
    height = int(opencv_config.getNode("image_height").real())
    width = int(opencv_config.getNode("image_width").real())
    print(f"Width: {width}, Height: {height}")

    size = (width, height)

    left_map_x, left_map_y, right_map_x, right_map_y = None, None, None, None
    left_node = opencv_config.getNode("left")
    right_node = opencv_config.getNode("right")
    left_K, left_dist = read_camera_intrinsics(left_node)
    right_K, right_dist = read_camera_intrinsics(right_node)

    if rectify:
        left_map_x, left_map_y = cv2.initUndistortRectifyMap(
            left_K, left_dist, np.eye(3), left_K, size, cv2.CV_32FC1
        )
        right_map_x, right_map_y = cv2.initUndistortRectifyMap(
            right_K, right_dist, np.eye(3), right_K, size, cv2.CV_32FC1
        )

    if scale != 1.0:
        size = (int(width * scale), int(height * scale))

    imu_file.write(
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
    )
    imu_file.write("\n")
    left_image_file.write("#timestamp [ns],filename\n")
    right_image_file.write("#timestamp [ns],filename\n")

    orb = open(os.path.join(dataset_folder, "timestamps.txt"), "w")
    for topic, msg, t in tqdm(bag.read_messages(), total=bag.get_message_count()):  # type: ignore
        stamp = str(msg.header.stamp.to_nsec())
        if topic == left_image_topic:
            if msg._type == "sensor_msgs/Image":
                left_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg._type == "sensor_msgs/CompressedImage":
                left_image = cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                raise ValueError("Unknown message type")

            orb.write(stamp + "\n")

            if rectify:
                left_image_undistorted = cv2.remap(
                    left_image,
                    left_map_x,
                    left_map_y,
                    interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )
            else:
                left_image_undistorted = left_image
            if scale != 1.0:
                left_image_undistorted = cv2.resize(left_image_undistorted, size)

            filename = os.path.join(left_image_data_folder, stamp + ".png")
            cv2.imwrite(filename, left_image_undistorted)
            left_image_file.write(stamp + "," + stamp + ".png" + "\n")

        elif topic == right_image_topic:
            if msg._type == "sensor_msgs/Image":
                right_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg._type == "sensor_msgs/CompressedImage":
                right_image = cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                raise ValueError("Unknown message type")
            if rectify:
                right_image_undistorted = cv2.remap(
                    right_image,
                    right_map_x,
                    right_map_y,
                    interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )
            else:
                right_image_undistorted = right_image

            if scale != 1.0:
                right_image_undistorted = cv2.resize(right_image_undistorted, size)

            filename = os.path.join(right_image_data_folder, stamp + ".png")
            cv2.imwrite(filename, right_image_undistorted)
            right_image_file.write(stamp + "," + stamp + ".png" + "\n")

        elif topic == imu_topic and msg._type == "sensor_msgs/Imu":
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
    config_file = rospy.get_param("~config_file")
    scale = rospy.get_param("~scale")
    rectify = False
    if rospy.has_param("~rectify"):
        rectify = rospy.get_param("~rectify")

    print("Euroc Folder: {}".format(dataset_folder))
    print("Input bag file: {}".format(bagfile))
    print("Config file: {}".format(config_file))
    print("Left image topic: {}".format(left_image_topic))
    print("Right image topic: {}".format(right_image_topic))
    print("Scale: {}".format(scale))
    print("IMU topic: {}".format(imu_topic))
    print(f"Rectify: {rectify}")

    extract_bag(
        dataset_folder,  # type: ignore
        bagfile,  # type: ignore
        left_image_topic,  # type: ignore
        right_image_topic,  # type: ignore
        imu_topic,  # type: ignore
        config_file,  # type: ignore
        scale,  # type: ignore
        rectify,  # type: ignore
    )
