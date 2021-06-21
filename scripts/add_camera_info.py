#!/usr/bin/python


import yaml

import rosbag
import rospy
from sensor_msgs.msg import Image, CameraInfo


class CamInfoWriter:

    def __init__(self, input_file, output_file, left_cam_topic,
                 left_info_file, right_cam_topic=None, right_info_file=None):

        # rospy.loginfo(' Processing input bagfile: %s', input_file)
        # self.inbag = rosbag.Bag(input_file, 'r')
        # rospy.loginfo(' Writing to output bagfile: %s', output_file)
        # self.outbag = rosbag.Bag(output_file, 'w')
        self.left_cam_info = self.readCamInfo(left_info_file)
        self.right_cam_info = self.readCamInfo(right_info_file)

    def __del__(self):
        self.inbag.close()
        self.outbag.close()

    def readCamInfo(self, cam_info_file):
        rospy.loginfo(
            "Reading camera info params from: {}".format(cam_info_file))
        with open(cam_info_file) as yaml_stream:
            calib_data = yaml.load(yaml_stream, Loader=yaml.FullLoader)
            print(calib_data)


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

    if use_stereo:
        rospy.loginfo("Populating camera info for stereo images...")
        if(not rospy.has_param('~right_cam_topic')):
            rospy.logfatal("Require right camera topic to populate cam info")
        right_cam_topic = rospy.get_param('~right_cam_topic')

        if(not rospy.has_param('~right_caminfo_file')):
            rospy.logfatal("Require right camera info file")
        right_cam_info_file = rospy.get_param('~right_caminfo_file')
    else:
        rospy.loginfo('Populating camera info for only left image...')

    # all_params = rospy.get_param_names()
    # for param_name in all_params:
    #     param_val = rospy.get_param(param_name)
    #     print('Found param: {} with value: {}'.format(param_name, param_val))

    cam_info_writer = CamInfoWriter(
        input_file, output_file, left_cam_topic, left_cam_info_file, right_cam_topic, right_cam_info_file)
