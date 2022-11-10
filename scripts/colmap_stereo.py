#!/usr/bin/env python

from configparser import Interpolation
import os
from turtle import stamp
import cv2

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np

from message_filters import Subscriber, TimeSynchronizer


def read_camera_intrinsics(cv_node: cv2.FileNode):
    # Read camera intrinsics from cv_node

    camera_matrix = np.ones((3, 3))
    distortion_coefficients = np.ones((1, 4))

    camera_matrix = cv_node.getNode("K").mat()
    distortion_coefficients = cv_node.getNode("D").mat()


    return camera_matrix, distortion_coefficients


class ColmapStereo:
    def __init__(self, root_folder, delay, scale=1.0, config_file=None):

        self.scale = scale
        self.bridge = CvBridge()

        left_image_sub = Subscriber("/camera0", CompressedImage)
        right_image_sub = Subscriber("/camera1", CompressedImage)

        if not os.path.exists(root_folder):
            os.makedirs(root_folder)
        image_folder = os.path.join(root_folder, "images")
        if not os.path.exists(image_folder):
            os.mkdir(image_folder)

        self.left_img_folder = os.path.join(image_folder, "left")
        if not os.path.exists(self.left_img_folder):
            os.mkdir(self.left_img_folder)

        self.right_img_folder = os.path.join(image_folder, "right")
        if not os.path.exists(self.right_img_folder):
            os.mkdir(self.right_img_folder)

        self.last_timestamp = -1
        self.delay = delay
        self.indx = 0

        opencv_config = cv2.FileStorage(config_file, cv2.FILE_STORAGE_READ)
        left_node = opencv_config.getNode("left")
        right_node = opencv_config.getNode("right")
        left_K, left_dist = read_camera_intrinsics(left_node)
        right_K, right_dist = read_camera_intrinsics(right_node)

        height = int(opencv_config.getNode("image_height").real())
        width = int(opencv_config.getNode("image_width").real())
        print(f"Width: {width}, Height: {height}")

        self.size = (width, height)

        self.left_map_x, self.left_map_y = cv2.initUndistortRectifyMap(
            left_K, left_dist, np.eye(3), left_K, self.size, cv2.CV_32FC1
        )
        self.right_map_x, self.right_map_y = cv2.initUndistortRectifyMap(
            right_K, right_dist, np.eye(3), right_K, self.size, cv2.CV_32FC1
        )

        ats = TimeSynchronizer([left_image_sub, right_image_sub], queue_size=10)
        ats.registerCallback(self.stereo_image_callback)

    def stereo_image_callback(self, left_img_msg, right_img_msg):
        rospy.logwarn_once(" !!! Inside Stereo Callback !!!")
        left_timestamp = left_img_msg.header.stamp.to_nsec()
        right_timestamp = right_img_msg.header.stamp.to_nsec()

        assert left_timestamp == right_timestamp
        stamp = left_timestamp
        if ((stamp - self.last_timestamp) * 1e-9) >= self.delay:

            left_image = self.bridge.compressed_imgmsg_to_cv2(
                left_img_msg, desired_encoding="bgr8"
            )

            left_image_undistorted = cv2.remap(
                left_image,
                self.left_map_x,
                self.left_map_y,
                interpolation=cv2.INTER_LINEAR,
            )
            cv2.imwrite(
                os.path.join(self.left_img_folder, "{}.png".format(str(stamp))),
                left_image_undistorted,
            )

            right_image = self.bridge.compressed_imgmsg_to_cv2(
                right_img_msg, desired_encoding="bgr8"
            )

            right_image_undistorted = cv2.remap(
                right_image,
                self.right_map_x,
                self.right_map_y,
                interpolation=cv2.INTER_LINEAR,
            )

            cv2.imwrite(
                os.path.join(self.right_img_folder, "{}.png".format(str(stamp))),
                right_image_undistorted,
            )

            self.last_timestamp = stamp
            self.indx += 1


if __name__ == "__main__":

    rospy.init_node("colmap_stereo", anonymous=True)

    if not rospy.has_param("~image_dir"):
        rospy.logfatal("Require the dataset path of asl directory")
        exit(1)

    if not rospy.has_param("~config_file"):
        rospy.logfatal("Require the config file path")
        exit(1)

    config_file = rospy.get_param("~config_file")

    scale = 1.0
    if rospy.has_param("~scale"):
        scale = rospy.get_param("~scale")

    image_folder = rospy.get_param("~image_dir")

    freq = 2.0
    if rospy.has_param("~img_freq"):
        freq = rospy.get_param("~img_freq")

    delay = 1.0 / freq  # type: ignore
    rospy.loginfo("Delay : {}".format(delay))

    colmap_stereo = ColmapStereo(
        image_folder, delay, scale=scale, config_file=config_file  # type: ignore
    )

    while not rospy.is_shutdown():
        rospy.spin()
