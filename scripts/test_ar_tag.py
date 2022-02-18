#!/usr/bin/python2

import rosbag
import tf.transformations as tfs
import numpy as np
import cv2

from collections import OrderedDict
from tqdm import tqdm
from cv_bridge import CvBridge


def get_3D_corners(min_x, max_x, min_y, max_y, min_z, max_z):
    corners = np.array([[min_x, min_y, min_z], [min_x, min_y, max_z],
                        [min_x, max_y, min_z], [min_x, max_y, max_z],
                        [max_x, min_y, min_z], [max_x, min_y, max_z],
                        [max_x, max_y, min_z], [max_x, max_y, max_z]])

    corners = np.concatenate((np.transpose(corners), np.ones((1, 8))), axis=0)
    return corners


# def get_camera_intrinsic():
#     K = np.array([[582.7585362687838, 0.0, 479.95702656023155],
#                   [0.0, 580.8446349178992, 271.8298929212255], [0.0, 0.0,
#                                                                 1.0]])
#     return K

# def get_distortion():
#     dist = np.array([
#         -0.1150974141409347, 0.10292772047643643, 0.001419836324816038,
#         -0.0018379214527896284
#     ]).astype(np.float32)
#     return dist


def get_camera_intrinsic():
    K = np.array([[583.16116134, 0.0, 481.92859774],
                  [0.0, 581.292988, 270.2504108], [0.0, 0.0, 1.0]])
    return K


def get_distortion():
    dist = np.array([-0.10973803, 0.09313709, 0.00143941,
                     0.0004831]).astype(np.float32)
    return dist


def compute_projection(points_3D, transformation, internal_calibration):
    projections_2d = np.zeros((2, points_3D.shape[1]), dtype='float32')
    camera_projection = (
        internal_calibration.dot(transformation)).dot(points_3D)
    projections_2d[0, :] = camera_projection[0, :] / camera_projection[2, :]
    projections_2d[1, :] = camera_projection[1, :] / camera_projection[2, :]
    return projections_2d


def draw_demo_img_corners(img,
                          projectpts,
                          color=(0, 255, 0),
                          nV=8,
                          thickness=2):

    vertices = []
    for i in range(nV):
        x = projectpts[i][0]
        y = projectpts[i][1]
        coordinates = (int(x), int(y))
        vertices.append(coordinates)
        cv2.circle(img, coordinates, 2, color, -1)

    # print(vertices)
    cv2.line(img, vertices[0], vertices[1], color, thickness=thickness)
    cv2.line(img, vertices[0], vertices[2], color, thickness=thickness)
    cv2.line(img, vertices[0], vertices[4], color, thickness=thickness)
    cv2.line(img, vertices[1], vertices[5], color, thickness=thickness)
    cv2.line(img, vertices[1], vertices[3], color, thickness=thickness)
    cv2.line(img, vertices[2], vertices[3], color, thickness=thickness)
    cv2.line(img, vertices[2], vertices[6], color, thickness=thickness)
    cv2.line(img, vertices[3], vertices[7], color, thickness=thickness)
    cv2.line(img, vertices[4], vertices[5], color, thickness=thickness)
    cv2.line(img, vertices[4], vertices[6], color, thickness=thickness)
    cv2.line(img, vertices[5], vertices[7], color, thickness=thickness)
    cv2.line(img, vertices[6], vertices[7], color, thickness=thickness)

    return img


if __name__ == '__main__':
    file = open('/home/bjoshi/utils_ws/gopro2_gennie_cavern3.txt', 'r')
    bagfile = '/media/bjoshi/data/gopro_bags/gopro2_gennie_cavern_3.bag'

    file.readline()
    lines = file.readlines()

    min_x = -0.1016
    min_y = -0.1016
    min_z = 0
    max_x = 0.1016
    max_y = 0.1016
    max_z = 0.5

    ar_tag_poses = OrderedDict()

    for line in lines:
        line = line.strip()
        line = line.split(',')
        stamp = line[0]
        id = int(line[1])
        pose = [float(x) for x in line[2:]]
        trans = pose[0:3]
        transform = tfs.quaternion_matrix(pose[3:])
        transform[0:3, 3] = trans

        ar_tag_poses[str(stamp)] = np.array(transform)

    corners = get_3D_corners(min_x, max_x, min_y, max_y, min_z, max_z)
    K = get_camera_intrinsic()
    D = get_distortion()
    dim = (960, 540)

    map1, map2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), K, dim,
                                             cv2.CV_16SC2)

    bag = rosbag.Bag(bagfile, 'r')
    cvbridge = CvBridge()

    for topic, msg, t in tqdm(
            bag.read_messages(topics=['/gopro/image_raw/compressed']),
            total=bag.get_message_count(
                topic_filters=['/gopro/image_raw/compressed'])):
        stamp = str(msg.header.stamp.to_nsec())
        img = cvbridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        undistorted_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        if stamp in ar_tag_poses:
            tf = ar_tag_poses[stamp][0:3, :]
            print(tf)
            bbox_3d = compute_projection(corners, tf, K)
            corners_2d = np.transpose(bbox_3d)
            img = draw_demo_img_corners(undistorted_img, corners_2d)
            cv2.imshow('image', undistorted_img)
            k = cv2.waitKey(0)
            if k == ord('q'):
                break
