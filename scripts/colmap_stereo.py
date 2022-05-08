#!/usr/bin/env python

import os
from turtle import stamp
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer


def read_camera_intrinsics(cv_node: cv2.FileNode):
    # Read camera intrinsics from cv_node

    camera_matrix = np.ones((3, 3))
    distortion_coefficients = np.ones((1, 4))

    camera_matrix = cv_node.getNode("K").mat()
    distortion_coefficients = cv_node.getNode('D').mat()

    print(camera_matrix)
    print(distortion_coefficients)

    return camera_matrix, distortion_coefficients


class ColmapStereo:

    def __init__(self, root_folder, delay, scale=1.0, config_file=None):

        self.scale = scale
        self.bridge = CvBridge()

        left_image_sub = Subscriber('/camera0', Image)
        right_image_sub = Subscriber('/camera1', Image)

        if not os.path.exists(root_folder):
            os.makedirs(root_folder)
        image_folder = os.path.join(root_folder, "images")
        if not os.path.exists(image_folder):
            os.mkdir(image_folder)

        self.left_img_folder = os.path.join(image_folder, 'left')
        if not os.path.exists(self.left_img_folder):
            os.mkdir(self.left_img_folder)

        self.right_img_folder = os.path.join(image_folder, 'right')
        if not os.path.exists(self.right_img_folder):
            os.mkdir(self.right_img_folder)

        self.last_timestamp = -1
        self.delay = delay
        self.indx = 0

        opencv_config = cv2.FileStorage(config_file, cv2.FILE_STORAGE_READ)
        left_node = opencv_config.getNode("left")
        right_node = opencv_config.getNode("right")
        self.left_K, self.left_dist = read_camera_intrinsics(left_node)
        self.right_K, self.right_dist = read_camera_intrinsics(right_node)

        ats = TimeSynchronizer(
            [left_image_sub, right_image_sub], queue_size=10)
        ats.registerCallback(self.stereo_image_callback)

    def stereo_image_callback(self, left_img_msg, right_img_msg):
        rospy.logwarn_once(" !!! Inside Stereo Callback !!!")
        left_timestamp = left_img_msg.header.stamp.to_nsec()
        right_timestamp = right_img_msg.header.stamp.to_nsec()
        h = left_img_msg.height
        w = left_img_msg.width

        new_h = int(h * self.scale)
        new_w = int(w * self.scale)
        size = (new_w, new_h)

        assert(left_timestamp == right_timestamp)
        stamp = left_timestamp
        if ((stamp - self.last_timestamp)*1e-9) >= self.delay:

            left_image = self.bridge.imgmsg_to_cv2(
                left_img_msg, desired_encoding='bgr8')

            camera_matrix = self.left_K
            if (self.scale != 1):
                left_image = cv2.resize(left_image, size)
                camera_matrix = self.scale * self.left_K
                camera_matrix[2, 2] = 1.0

            left_image_undistorted = cv2.undistort(
                left_image, camera_matrix, self.left_dist, None, camera_matrix)
            cv2.imwrite(os.path.join(self.left_img_folder,
                                     '{}.png'.format(str(stamp))), left_image_undistorted)

            right_image = self.bridge.imgmsg_to_cv2(
                right_img_msg, desired_encoding='bgr8')

            camera_matrix = self.right_K
            if (self.scale != 1):
                right_image = cv2.resize(right_image, size)
                camera_matrix = self.scale * self.right_K
                camera_matrix[2, 2] = 1.0

            right_image_undistorted = cv2.undistort(
                right_image, camera_matrix, self.right_dist, None, camera_matrix)
            cv2.imwrite(os.path.join(self.right_img_folder,
                                     '{}.png'.format(str(stamp))), right_image_undistorted)

            self.last_timestamp = stamp
            self.indx += 1


if __name__ == '__main__':

    rospy.init_node('colmap_stereo', anonymous=True)

    if (not rospy.has_param('~image_dir')):
        rospy.logfatal("Require the dataset path of asl directory")
        exit(1)

    if(not rospy.has_param('~config_file')):
        rospy.logfatal("Require the config file path")
        exit(1)

    config_file = rospy.get_param('~config_file')

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    image_folder = rospy.get_param('~image_dir')

    freq = 2.0
    if (rospy.has_param('~img_freq')):
        freq = rospy.get_param('~img_freq')

    delay = 1.0/freq
    rospy.loginfo("Delay : {}".format(delay))

    colmap_stereo = ColmapStereo(
        image_folder, delay, scale=scale, config_file=config_file)

    while not rospy.is_shutdown():
        rospy.spin()
