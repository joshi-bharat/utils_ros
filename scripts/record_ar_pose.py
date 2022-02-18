#!/usr/bin/python
'''
Author: Sharmin Rahman
Date created: 10/28/2019

- Subscribes to the 

- To run: python record_ar_pose.py

'''

#import argparse
import sys
import rospy

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from rospy.impl.transport import OUTBOUND


class record_pose:
    def __init__(self, marker_topic, filename):
        self.out_file = open(filename, 'w')
        self.out_file.write("stamp, id, px, py, pz, qx, qy, qz, qw\n")
        self.sub = rospy.Subscriber(marker_topic, AlvarMarkers,
                                    self.ar_marker_sub)
        self.count = 0

    def __del__(self):
        rospy.loginfo("Total tags: {}", self.count)
        self.out_file.close()

    def ar_marker_sub(self, ar_msgs):

        for msg in ar_msgs.markers:
            ar_id = msg.id
            position = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            self.out_file.write('{},{},{},{},{},{},{},{},{}\n'.format(
                str(msg.header.stamp), str(ar_id), str(position.x),
                str(position.y), str(position.z), str(quat.x), str(quat.y),
                str(quat.z), str(quat.w)))
            self.count += 1

        rospy.loginfo_throttle(60, "Total tags: {}".format(self.count))


if __name__ == '__main__':
    #parser = argparse.ArgumentParser(description="Record pose from ar_track_alvar package")
    #parser.add_argument('--file', help='file to record pose')
    #args = parser.parse_args()
    #filename = args.file

    rospy.init_node("ar_pose_recorder")

    if not rospy.has_param('~marker_topic'):
        rospy.logfatal('marker topic required')
        sys.exit()

    if not rospy.has_param('~output_file'):
        rospy.logfatal('output_file')
        sys.exit()

    marker_topic = rospy.get_param('~marker_topic')
    filename = rospy.get_param('~output_file')

    pose_recorder = record_pose(marker_topic, filename)
    rospy.spin()
