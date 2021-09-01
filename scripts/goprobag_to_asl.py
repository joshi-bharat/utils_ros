#!/usr/bin/python

import rospy
import rosbag
from rospy.core import logfatal
from tqdm import tqdm
import os
from cv_bridge import CvBridge
import cv2


def extract_bag(dataset_folder, bagfile, image_topic, imu_topic, start_time,
                end_time):

    bag = rosbag.Bag(bagfile, 'r')
    image_folder = os.path.join(dataset_folder, 'cam0')
    imu_folder = os.path.join(dataset_folder, 'imu0')

    if not os.path.exists(image_folder):
        os.makedirs(image_folder)
    if not os.path.exists(imu_folder):
        os.makedirs(imu_folder)

    image_file = open(os.path.join(image_folder, 'data.csv'), 'w')
    imu_file = open(os.path.join(imu_folder, 'data.csv'), 'w')

    image_data_folder = os.path.join(image_folder, 'data')
    if not os.path.exists(image_data_folder):
        os.makedirs(image_data_folder)

    topics = [image_topic, imu_topic]

    cv_bridge = CvBridge()

    imu_file.write(
        '#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]'
    )
    imu_file.write('\n')
    image_file.write('#timestamp [ns],filename\n')

    orb = open(os.path.join(dataset_folder, 'timestamps.txt'), 'w')
    start_time = bag.get_start_time() + start_time
    if end_time != -1:
        end_time = bag.get_start_time() + end_time

    for topic, msg, t in tqdm(bag.read_messages(),
                              total=bag.get_message_count()):
        stamp = str(msg.header.stamp.to_nsec())
        filename = os.path.join(image_data_folder, stamp + '.png')

        if t.to_sec() <= start_time or t.to_sec() >= end_time:
            continue

        if topic in topics:
            if msg._type == 'sensor_msgs/Image':
                cv_image = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(filename, cv_image)
                image_file.write(stamp + ',' + stamp + '.png' + '\n')
                orb.write(stamp + '\n')
            elif msg._type == 'sensor_msgs/CompressedImage':
                cv_image = cv_bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(filename, cv_image)
                image_file.write(stamp + ',' + stamp + '.png' + '\n')
                orb.write(stamp + '\n')
            elif msg._type == 'sensor_msgs/Imu':
                acc = msg.linear_acceleration
                ang_vel = msg.angular_velocity
                imu_file.write('{},{},{},{},{},{},{}\n'.format(
                    stamp, ang_vel.x, ang_vel.y, ang_vel.z, acc.x, acc.y,
                    acc.z))
    imu_file.close()
    image_file.close()
    bag.close()


if __name__ == '__main__':
    rospy.init_node('gopro_bag_to_asl')

    if (not rospy.has_param('~dataset_path')):
        rospy.logfatal("Require the dataset path of asl directory")

    if (not rospy.has_param('~bag')):
        rospy.logfatal("Require the bag file for extraction")

    if (not rospy.has_param('~image_topic')):
        rospy.logfatal("Require the image_topic")

    if (not rospy.has_param('~imu_topic')):
        rospy.logfatal("Require the imu topic")

    dataset_folder = rospy.get_param('~dataset_path')
    bagfile = rospy.get_param('~bag')
    image_topic = rospy.get_param('~image_topic')
    imu_topic = rospy.get_param('~imu_topic')

    start_time = 0
    end_time = -1
    if rospy.has_param('~start_time'):
        start_time = rospy.get_param('~start_time')
    if rospy.has_param('~end_time'):
        end_time = rospy.get_param('~end_time')

    extract_bag(dataset_folder, bagfile, image_topic, imu_topic, start_time,
                end_time)
