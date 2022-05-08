#!/usr/bin/env python
from pprint import pprint
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

from message_filters import Subscriber, ApproximateTimeSynchronizer, TimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError

K1 = np.array([[1155.5791586674245, 0.0, 789.8900364485113],
               [0.0, 1154.3388762071647, 556.1284087786863],
               [0.0, 0.0, 1.0]])
K2 = np.array([[1157.0636301971845, 0.0, 792.3728034861113],
               [0.0, 1155.9218909281092, 544.8458620306955],
               [0.0, 0.0, 1.0]])
D1 = np.array([0.1746140323149177, 0.40819325039769905,
               0.005101656278726976, -0.0071643416533231875])
D2 = np.array([0.16250643068169784, 0.4596504111879903,
               0.004150626585386907, -0.0036379497012914894])

R = np.array([[0.9998992467988412, 0.004452798092094777, -0.013478458378276771],
              [-0.004439424726289592, 0.9999896234989181, 0.0010219601683559491],
              [0.013482869101326718, -0.0009620206012007464, 0.9999086392051793]])

T = np.array(
    [-0.10848623399569293, 0.00019887889874828634, 0.00027577792198759304])
Size = (1600, 1100)
R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(cameraMatrix1=K1, cameraMatrix2=K2,  distCoeffs1=D1,
                                                           distCoeffs2=D2, imageSize=Size,  R=R, T=T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

map_left_x, map_left_y = cv2.initUndistortRectifyMap(
    K1, D1, R1, P1, Size, cv2.CV_32FC1)
map_right_x, map_right_y = cv2.initUndistortRectifyMap(
    K2, D2, R2, P2, Size, cv2.CV_32FC1)


bridge = CvBridge()


def stereo_callback(left_image_msg, right_image_msg):
    left_image = bridge.compressed_imgmsg_to_cv2(
        left_image_msg, desired_encoding='bgr8')
    right_image = bridge.compressed_imgmsg_to_cv2(
        right_image_msg, desired_encoding='bgr8')

    left_rectified = cv2.remap(left_image,
                               map_left_x, map_left_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    right_rectified = cv2.remap(right_image,
                                map_right_x, map_right_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

    left_rectified = cv2.resize(left_rectified, (800, 550))
    right_rectified = cv2.resize(right_rectified, (800, 550))

    img = cv2.hconcat([left_rectified, right_rectified])
    cv2.imshow('stereo', img)
    cv2.waitKey(1)


if __name__ == '__main__':

    rospy.init_node('stereo_test', anonymous=True)
    left_image_sub = Subscriber(
        "/stereorig/left/image_raw/compressed", CompressedImage)
    right_image_sub = Subscriber(
        "/stereorig/right/image_raw/compressed", CompressedImage)
    exact_time_sync = TimeSynchronizer([left_image_sub, right_image_sub], 100)
    exact_time_sync.registerCallback(stereo_callback)

    rospy.spin()
    cv2.destroyAllWindows()
