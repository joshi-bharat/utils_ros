#! /usr/bin/env python

from __future__ import print_function
import os
import cv2
from cv_bridge import CvBridge

import rospy
import rosbag
import message_filters
from tqdm import tqdm

from sensor_msgs.msg import Image, CompressedImage


class ImageExtractor():
    def __init__(self,
                 image_dir: str,
                 bag_file: str,
                 stereo: bool,
                 compressed: bool,
                 left_topic: str,
                 cache_size: int = 1000,
                 slop: float = 0.02,
                 right_topic: str = None) -> None:

        self.stereo = stereo
        self.bag = rosbag.Bag(bag_file, 'r')
        self.img_dir = image_dir
        self.left_image_dir = os.path.join(self.img_dir, 'left')
        self.right_image_dir = os.path.join(self.img_dir, 'right')

        if (not os.path.exists(image_dir)):
            os.makedirs(image_dir)

        if self.stereo:
            if compressed:
                self.message_types = [CompressedImage] * 2
                self.img_topics = [
                    left_topic + '/compressed', right_topic + '/compressed'
                ]
            else:
                self.img_topics = [left_topic, right_topic]
                self.message_types = [Image] * 2
            os.mkdir(self.left_image_dir)
            os.mkdir(self.right_image_dir)
        else:
            if compressed:
                self.img_topics = [left_topic + '/compressed']
                self.message_types = [CompressedImage]
            else:
                self.img_topics = [left_topic + '/compressed']
                self.message_types = [Image]

        self.synchronizer = None
        self.filters = None
        if self.stereo:
            self.filters = [
                message_filters.SimpleFilter() for _ in self.img_topics
            ]
            self.synchronizer = message_filters.TimeSynchronizer(
                self.filters, cache_size, slop)
            self.synchronizer.registerCallback(self.write_stereo_images)

        self.cv_bridge = CvBridge()

    def __del__(self):
        self.bag.close()

    def save_image(self, img_msg, base_dir: str):
        stamp = str(img_msg.header.stamp.to_nsec())
        filename = os.path.join(base_dir, stamp + '.png')
        if img_msg._type == "sensor_msgs/CompressedImage":
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg)
        else:
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        cv2.imwrite(filename, cv_image)

    def extract_images(self, delay: float = 0.0):
        start_time = self.bag.get_start_time()
        previous_stamp = self.bag.get_start_time()

        print("topics: {}".format(self.img_topics))
        for topic, msg, t in tqdm(self.bag.read_messages(),
                                  total=self.bag.get_message_count()):
            if topic in self.img_topics:
                if t.to_sec() < start_time:
                    continue
                if t.to_sec() - previous_stamp < delay:
                    continue
                if self.stereo:
                    self.filters[self.img_topics.index(topic)].signalMessage(
                        msg)
                else:
                    self.save_image(msg, self.img_dir)
                previous_stamp = t.to_sec()

    def write_stereo_images(self, left_msg, right_msg):
        # Matches timestamps in left and right messages
        new_stamp = rospy.Time(nsecs=min(left_msg.header.stamp.to_nsec(),
                                         right_msg.header.stamp.to_nsec()))

        left_msg.header.stamp = new_stamp
        right_msg.header.stamp = new_stamp
        self.save_image(left_msg, self.left_image_dir)
        self.save_image(right_msg, self.right_image_dir)


if __name__ == "__main__":

    rospy.init_node('extract_images', anonymous=True)

    if (not rospy.has_param('~image_dir')):
        rospy.logfatal("Require the dataset path of asl directory")

    if (not rospy.has_param('~bag')):
        rospy.logfatal("Require the bag file to extract images")

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    image_folder = rospy.get_param('~image_dir')
    bag_file = rospy.get_param('~bag')

    delay = 0.0
    if (rospy.has_param('~write_every_nsecs')):
        delay = rospy.get_param('~write_every_nsecs')

    stereo = False
    if (rospy.has_param('~stereo')):
        stereo = rospy.get_param('~stereo')

    start = 0.0
    if (rospy.has_param('~start')):
        to_secs = rospy.get_param('~upto_n_secs')

    slop = 0.02
    if (rospy.has_param('~slop')):
        slop = rospy.get_param('~slop')

    cache_size = 100
    if (rospy.has_param('~cache_size')):
        cache_size = rospy.get_param('~cache_size')

    left = "/left/image_raw"
    if (rospy.has_param('~left')):
        left = rospy.get_param('~left')

    right = "/right/image_raw"
    if (rospy.has_param('~right')):
        right = rospy.get_param('~right')

    compressed = False
    if (rospy.has_param('~compressed')):
        compressed = rospy.get_param('~compressed')

    rospy.loginfo("bag: {}".format(bag_file))
    rospy.loginfo("image dir: {}".format(image_folder))
    rospy.loginfo("Delay : {}".format(delay))
    rospy.loginfo("stereo: {}".format(stereo))
    rospy.loginfo("compressed: {}".format(compressed))
    rospy.loginfo("left image topic: {}".format(left))
    rospy.loginfo("right image topic: {}".format(right))

    extractor = ImageExtractor(image_dir=image_folder,
                               bag_file=bag_file,
                               stereo=stereo,
                               left_topic=left,
                               right_topic=right,
                               compressed=compressed,
                               cache_size=cache_size,
                               slop=slop)
    extractor.extract_images(delay=delay)

    # while not rospy.is_shutdown():
    #     rospy.spin()
