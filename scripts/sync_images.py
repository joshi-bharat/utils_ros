#!/usr/bin/env python

import os
import cv2

import rospy
import rosbag
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from message_filters import ApproximateTimeSynchronizer, Subscriber


class BagSync:

    def __init__(self, rosbag_filename, scale=1.0, compress_image=False):

        self.scale = scale
        self.compress_images = compress_image
        self.bridge = CvBridge()

        left_image_sub = Subscriber('/camera0', Image)
        right_image_sub = Subscriber('/camera1', Image)

        ats = ApproximateTimeSynchronizer(
            [left_image_sub, right_image_sub], queue_size=20, slop=0.2)
        ats.registerCallback(self.stereo_image_callback)
        rospy.Subscriber('/imu', Imu, self.imu_sub, queue_size=100)

        self.last_timestamp = -1
        self.outbag = rosbag.Bag(rosbag_filename, 'w')

        self.left_img_topic = "/cam0/image_raw"
        self.right_img_topic = "/cam1/image_raw"
        self.imu_topic = "/imu0"

    def __del__(self):
        self.outbag.close()

    def imu_sub(self, imu_msg):
        rospy.logwarn_once("!!! Inside IMU Callback !!!")
        self.outbag.write(self.imu_topic, imu_msg, imu_msg.header.stamp)

    def stereo_image_callback(self, left_img_msg, right_img_msg):
        rospy.logwarn_once(" !!! Inside Stereo Callback !!!")
        left_timestamp = left_img_msg.header.stamp.to_nsec()
        right_timestamp = right_img_msg.header.stamp.to_nsec()
        h = left_img_msg.height
        w = left_img_msg.width

        new_h = int(h * self.scale)
        new_w = int(w * self.scale)
        size = (new_w, new_h)

        if (left_timestamp > self.last_timestamp and right_timestamp > self.last_timestamp):

            if(left_timestamp < right_timestamp):
                stamp = right_timestamp
                ros_time = right_img_msg.header.stamp
            else:
                stamp = left_timestamp
                ros_time = left_img_msg.header.stamp

            # rospy.loginfo("Stamp: {}".format(ros_time.to_nsec()))
            left_image = self.bridge.imgmsg_to_cv2(
                left_img_msg, desired_encoding='passthrough')
            if (self.scale != 1):
                left_image = cv2.resize(left_image, size)

            right_image = self.bridge.imgmsg_to_cv2(
                right_img_msg, desired_encoding='passthrough')

            if (self.scale != 1):
                right_image = cv2.resize(right_image, size)

            if(self.compress_images):
                left_comp_img = self.bridge.cv2_to_compressed_imgmsg(
                    left_image)
                right_comp_img = self.bridge.cv2_to_compressed_imgmsg(
                    right_image)
                left_comp_img.header = left_img_msg.header
                left_comp_img.header.stamp = ros_time
                right_comp_img.header = right_img_msg.header
                right_comp_img.header.stamp = ros_time

                self.outbag.write(self.left_img_topic+"/compressed",
                                  left_comp_img, ros_time)
                self.outbag.write(self.right_img_topic+"/compressed",
                                  right_comp_img, ros_time)
            else:
                left_sync_msg = self.bridge.cv2_to_imgmsg(left_image)
                right_sync_msg = self.bridge.cv2_to_imgmsg(right_image)
                left_sync_msg.header = left_img_msg.header
                left_sync_msg.header.stamp = ros_time
                right_sync_msg.header = right_img_msg.header
                right_sync_msg.header.stamp = ros_time

                self.outbag.write(self.left_img_topic+"/compressed",
                                  left_sync_msg, ros_time)
                self.outbag.write(self.right_img_topic+"/compressed",
                                  right_sync_msg, ros_time)
            self.last_timestamp = stamp

        else:
            rospy.logwarn(
                'Left Stamp: {} or Right Stamp: {} not greater than the previous stamp: {}'.format(
                    left_timestamp, right_timestamp, self.last_timestamp))


if __name__ == '__main__':

    rospy.init_node('bag_sync_images', anonymous=True)

    # all_params = rospy.get_param_names()
    # for param in all_params:
    #     rospy.logwarn(param)

    if (not rospy.has_param('~rosbag_filename')):
        rospy.logfatal("Require the dataset path of asl directory")

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    compress_image = False
    if (rospy.has_param('~compress_image')):
        compress_image = rospy.get_param('~compress_image')

    rosbag_filename = rospy.get_param('~rosbag_filename')

    asl_converter = BagSync(
        rosbag_filename, scale=scale, compress_image=compress_image)

    while not rospy.is_shutdown():
        rospy.spin()
