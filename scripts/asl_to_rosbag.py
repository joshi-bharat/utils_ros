#!/usr/bin/python2


from tqdm import tqdm
import glob
import cv2
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, Imu
import rospy
import rosbag
import os


class ASLtoRosBag:

    def __init__(self, dataset_path, bagfile, scale=1.0, write_stereo=True, write_imu=True):
        self.dataset_path = os.path.join(dataset_folder, 'mav0')
        self.bag = rosbag.Bag(bagfile, 'w')

        self.cam0_topic = '/cam0/image_raw'
        self.cam1_topic = '/cam1/image_raw'
        self.imu_topic = '/imu0'

        self.use_stereo = write_stereo
        self.use_imu = write_imu

        self.finished = False

    def __del__(self):
        self.bag.close()

    def write_left_cam(self):

        rospy.loginfo("Writing left camera msg to bag file")
        cam0_folder = os.path.join(self.dataset_path, "cam0", "data")
        indx = 0
        files = glob.glob(cam0_folder+"/*")
        files = sorted(files)

        for file in tqdm(files):
            stamp = os.path.split(file)[1].split('.')[0]
            secs = int(float(stamp) * 1e-9)
            n_secs = int(float(stamp) - secs*1e9)

            ros_time = rospy.Time(secs, n_secs)

            header = Header()
            header.stamp = ros_time
            header.frame_id = 'cam0'
            header.seq = indx

            image = cv2.imread(file)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            bridge = CvBridge()
            msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            msg.header = header
            msg.encoding = 'mono8'
            self.bag.write(self.cam0_topic, msg, header.stamp)
            indx += 1

    def write_right_cam(self):
        rospy.loginfo("Writing right camera msg to bag file")
        cam1_folder = os.path.join(self.dataset_path, "cam1", "data")
        indx = 0
        files = glob.glob(cam1_folder+"/*")
        files = sorted(files)
        for file in tqdm(files):
            stamp = os.path.split(file)[1].split('.')[0]
            secs = int(float(stamp) * 1e-9)
            n_secs = int(float(stamp) - secs*1e9)

            ros_time = rospy.Time(secs, n_secs)

            header = Header()
            header.stamp = ros_time
            header.frame_id = 'cam1'
            header.seq = indx

            image = cv2.imread(file)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            bridge = CvBridge()
            msg = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            msg.header = header
            msg.encoding = 'mono8'
            self.bag.write(self.cam1_topic, msg, header.stamp)
            indx += 1

    def write_imu_msg(self):

        rospy.loginfo("Writing imu msg to bag file")
        imu_file = open(os.path.join(self.dataset_path, "imu0", "data.csv"))
        indx = 0
        # skip the first line asl format
        imu_file.readline()
        lines = imu_file.readlines()
        for line in lines:
            line = line.strip()
            line_arr = line.split(',')
            stamp = line_arr[0]
            secs = int(float(stamp) * 1e-9)
            n_secs = int(float(stamp) - secs*1e9)
            ros_time = rospy.Time(secs, n_secs)

            header = Header()
            header.stamp = ros_time
            header.frame_id = 'imu0'
            header.seq = indx

            angular_vel = Vector3()
            angular_vel.x = float(line_arr[1])
            angular_vel.y = float(line_arr[2])
            angular_vel.z = float(line_arr[3])

            linear_acc = Vector3()
            linear_acc.x = float(line_arr[4])
            linear_acc.y = float(line_arr[5])
            linear_acc.z = float(line_arr[6])

            imu = Imu()
            imu.header = header
            imu.angular_velocity = angular_vel
            imu.linear_acceleration = linear_acc

            self.bag.write(self.imu_topic, imu, header.stamp)
            indx += 1

    def write_bag(self):
        self.write_left_cam()
        if(self.use_stereo):
            self.write_right_cam()
        if(self.use_imu):
            self.write_imu_msg()
        self.finished = True


if __name__ == "__main__":

    rospy.init_node('asl_to_rosbag', anonymous=True)

    if (not rospy.has_param('~dataset_path')):
        rospy.logfatal("Require the dataset path of asl directory")

    if (not rospy.has_param('~bagfile')):
        rospy.logfatal("Require the bagfile to be written")

    dataset_folder = rospy.get_param('~dataset_path')
    rosbag_filename = rospy.get_param('~bagfile')

    scale = 1.0
    if (rospy.has_param('~scale')):
        scale = rospy.get_param('~scale')

    use_stereo = True
    if (rospy.has_param('~use_stereo')):
        use_stereo = rospy.get_param('~use_stereo')

    use_imu = True
    if (rospy.has_param('~use_imu')):
        use_stereo = rospy.get_param('~use_imu')

    bag_writer = ASLtoRosBag(dataset_folder, rosbag_filename)
    bag_writer.write_bag()
