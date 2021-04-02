#!/usr/bin/env python

import os
import cv2

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from message_filters import Subscriber


class ASLConverter:

    def __init__(self, image_dir, delay, scale=1.0):

        self.scale = scale
        self.bridge = CvBridge()

        image_sub = Subscriber('/camera0', Image)
        image_sub.registerCallback(self.left_image_sub)
 
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)

        self.image_dir = image_dir
        rospy.loginfo("COLMAP image directory: {}".format(self.image_dir) )

        self.last_timestamp = -1
        self.delay = delay


    def left_image_sub(self, img_msg):
        rospy.logwarn_once('!!!Inside the left image callback !!!')
        h = img_msg.height
        w = img_msg.width

        new_h = int(h * self.scale)
        new_w = int(w * self.scale)
        size = (new_w, new_h)
        cur_time = img_msg.header.stamp.to_sec()
    
        if(cur_time - self.last_timestamp) >= delay:
            cv_image = self.bridge.imgmsg_to_cv2(
                img_msg, desired_encoding='passthrough')
            if self.scale != 1.0:
                cv_image = cv2.resize(cv_image, size)
            
            cv2.imwrite(os.path.join(self.image_dir,
                                 '{}.png'.format(str(img_msg.header.stamp))), cv_image)
            self.last_timestamp = cur_time




if __name__ == '__main__':

    rospy.init_node('bal_to_colmap', anonymous=True)

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
    asl_converter = ASLConverter(image_folder, delay, scale=scale)

    while not rospy.is_shutdown():
        rospy.spin()
