#!/usr/bin/python


import yaml

import rosbag
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class CamInfoWriter:

    def __init__(self, input_file, output_file, left_cam_topic,
                 left_info_file, compressed=False, right_cam_topic=None, right_info_file=None,
                 left_cam_frame='', right_cam_frame=''):

        self.left_cam_info = self.readCamInfo(left_info_file)
        self.right_cam_info = self.readCamInfo(right_info_file)

        rospy.loginfo("!!!!!!!!!!!!!!! Left Camera Info !!!!!!!!!!!!!!!")
        print(self.left_cam_info)

        rospy.loginfo("\n!!!!!!!!!!!!!!! Right Camera Info !!!!!!!!!!!!!!!")
        print(self.right_cam_info)

        rospy.loginfo(' Processing input bagfile: %s', input_file)
        self.inbag = rosbag.Bag(input_file, 'r')
        rospy.loginfo(' Writing to output bagfile: %s', output_file)
        self.outbag = rosbag.Bag(output_file, 'w')

        self.left_cam_topic = left_cam_topic
        self.right_cam_topic = right_cam_topic
        self.left_cam_info_topic = self.left_cam_topic + "/camera_info"
        self.right_cam_info_topic = self.right_cam_topic + "/camera_info"

        self.left_cam_link = left_cam_frame
        self.right_cam_link = right_cam_frame

        if compressed:
            self.left_cam_topic = self.left_cam_topic + '/compressed'
            self.right_cam_topic = self.right_cam_topic + '/compressed'

        self.left_indx = 0
        self.right_indx = 0

        self.writeCameraInfo()

    def __del__(self):
        self.inbag.close()
        self.outbag.close()

    def readCamInfo(self, cam_info_file):
        rospy.loginfo(
            "Reading camera info params from: {}".format(cam_info_file))
        yaml_stream = open(cam_info_file)
        calib_data = yaml.load(yaml_stream, Loader=yaml.FullLoader)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']

        cam_info.distortion_model = calib_data['distortion_model']
        cam_info.D = calib_data['distortion_coefficients']

        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']

        # Some defualt extra params
        cam_info.binning_x = int(1)
        cam_info.binning_y = int(1)

        # No need to set roi.x_offset or roi.y_offset. Both default to 0
        cam_info.roi.height = cam_info.height
        cam_info.roi.width = cam_info.width
        cam_info.roi.do_rectify = False

        yaml_stream.close()

        return cam_info

    def writeCameraInfo(self):
        for topic, msg, t in self.inbag.read_messages():
            if topic == self.left_cam_topic:
                header = Header()
                header.seq = self.left_indx
                header.stamp = msg.header.stamp
                header.frame_id = self.left_cam_link
                self.left_cam_info.header = header
                self.outbag.write(self.left_cam_info_topic,
                                  self.left_cam_info, header.stamp)
                self.left_indx += 1
            elif topic == self.right_cam_topic:
                header = Header()
                header.seq = self.right_indx
                header.stamp = msg.header.stamp
                header.frame_id = self.right_cam_link
                self.right_cam_info.header = header
                self.outbag.write(self.right_cam_info_topic,
                                  self.right_cam_info, header.stamp)
                self.right_indx += 1

            self.outbag.write(topic, msg, t)

        rospy.loginfo('Closing output bagfile and exit...')
        self.outbag.close()
        self.inbag.close()


if __name__ == '__main__':

    rospy.init_node('add_cam_info', anonymous=True)

    if (not rospy.has_param('~input_bagfile')):
        rospy.logfatal(
            "Require the input bag file with camera message")
        dataset_folder = rospy.get_param('~dataset_path')
    input_file = rospy.get_param('~input_bagfile')

    if (not rospy.has_param('~output_bagfile')):
        rospy.logfatal("Require the bagfile to be written")
    output_file = rospy.get_param('~output_bagfile')

    if(not rospy.has_param('~left_cam_topic')):
        rospy.logfatal("Require left camera topic to populate cam info")
    left_cam_topic = rospy.get_param('~left_cam_topic')

    if(not rospy.has_param('~left_caminfo_file')):
        rospy.logfatal("Require left camera info file")
    left_cam_info_file = rospy.get_param('~left_caminfo_file')

    right_cam_topic = None
    right_cam_info_file = None

    use_stereo = False
    if (rospy.has_param('~use_stereo')):
        use_stereo = rospy.get_param('~use_stereo')

    left_cam_frame = ''
    right_cam_frame = ''
    if rospy.has_param('~left_cam_frame'):
        left_cam_frame = rospy.get_param('~left_cam_frame')

    if use_stereo:
        rospy.loginfo("Populating camera info for stereo images...")
        if(not rospy.has_param('~right_cam_topic')):
            rospy.logfatal("Require right camera topic to populate cam info")
        right_cam_topic = rospy.get_param('~right_cam_topic')

        if(not rospy.has_param('~right_caminfo_file')):
            rospy.logfatal("Require right camera info file")
        right_cam_info_file = rospy.get_param('~right_caminfo_file')

        if rospy.has_param('~right_cam_frame'):
            right_cam_frame = rospy.get_param('~right_cam_frame')
    else:
        rospy.loginfo('Populating camera info for only left image...')

    # all_params = rospy.get_param_names()
    # for param_name in all_params:
    #     param_val = rospy.get_param(param_name)
    #     print('Found param: {} with value: {}'.format(param_name, param_val))

    compressed = False
    if (rospy.has_param('~compressed')):
        compressed = rospy.get_param('~compressed')

    cam_info_writer = CamInfoWriter(
        input_file, output_file, left_cam_topic, left_cam_info_file, compressed, right_cam_topic,
        right_cam_info_file, left_cam_frame, right_cam_frame)
