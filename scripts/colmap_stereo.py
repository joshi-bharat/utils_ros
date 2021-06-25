#!/usr/bin/env python

import os
import cv2

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from message_filters import ApproximateTimeSynchronizer, Subscriber


class ColmapStereo:

    def __init__(self, root_folder, delay, scale=1.0,):

        self.scale = scale
        self.bridge = CvBridge()

        left_image_sub = Subscriber('/camera0', Image)
        right_image_sub = Subscriber('/camera1', Image)

        ats = ApproximateTimeSynchronizer(
            [left_image_sub, right_image_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.stereo_image_callback)

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


    def stereo_image_callback(self, left_img_msg, right_img_msg):
        rospy.logwarn_once(" !!! Inside Stereo Callback !!!")
        left_timestamp = left_img_msg.header.stamp.to_nsec()
        right_timestamp = right_img_msg.header.stamp.to_nsec()
        h = left_img_msg.height
        w = left_img_msg.width

        new_h = int(h * self.scale)
        new_w = int(w * self.scale)
        size = (new_w, new_h)

        stamp = max(left_timestamp, right_timestamp)

        if ((stamp - self.last_timestamp)*1e-9) >= self.delay:

            left_image = self.bridge.imgmsg_to_cv2(
                left_img_msg, desired_encoding='passthrough')
            if (self.scale != 1):
                left_image = cv2.resize(left_image, size)
            cv2.imwrite(os.path.join(self.left_img_folder,
                                     '{}.png'.format("L_"+str(stamp))), left_image)

            right_image = self.bridge.imgmsg_to_cv2(
                right_img_msg, desired_encoding='passthrough')

            if (self.scale != 1):
                right_image = cv2.resize(right_image, size)
            cv2.imwrite(os.path.join(self.right_img_folder,
                                     '{}.png'.format("R_"+str(stamp))), right_image)

            self.last_timestamp = stamp
            self.indx += 1


if __name__ == '__main__':

    rospy.init_node('colmap_stereo', anonymous=True)

    if (not rospy.has_param('~image_dir')):
        rospy.logfatal("Require the dataset path of asl directory")

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    image_folder = rospy.get_param('~image_dir')

    freq = 2.0
    if (rospy.has_param('~img_freq')):
        freq = rospy.get_param('~img_freq')
    
    delay = 1.0/freq
    rospy.loginfo("Delay : {}".format(delay))
    asl_converter = ColmapStereo(image_folder, delay, scale=scale)

    while not rospy.is_shutdown():
        rospy.spin()