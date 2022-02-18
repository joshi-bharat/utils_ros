#! /usr/bin/env python2

from __future__ import print_function

import rospy
import rosbag
import message_filters
import argparse
from tqdm import tqdm
from collections import OrderedDict


def sync(left_msg, right_msg, left_info, right_info):
    # Matches timestamps in left and right messages
    new_stamp = rospy.Time(nsecs=min(left_msg.header.stamp.to_nsec(),
                           right_msg.header.stamp.to_nsec()))

    if left_msg.header.stamp != left_info.header.stamp:
        print("WARN Mismatched left camera info. Image: %s, Camera Info: %s, Difference: %s"
              % (left_msg.header.stamp, left_info.header.stamp,
                 abs(left_msg.header.stamp.to_sec() - left_info.header.stamp.to_sec())))
    if right_msg.header.stamp != right_info.header.stamp:
        print("WARN Mismatched right camera info. Image: %s, Camera Info: %s, Difference: %s"
              % (right_msg.header.stamp, right_info.header.stamp,
                 abs(right_msg.header.stamp.to_sec() - right_info.header.stamp.to_sec())))

    left_msg.header.stamp = new_stamp
    right_msg.header.stamp = new_stamp
    left_info.header.stamp = new_stamp
    right_info.header.stamp = new_stamp

    return left_msg, right_msg, left_info, right_info


def sync(left_msg, right_msg):
    # Matches timestamps in left and right messages
    new_stamp = rospy.Time(nsecs=min(left_msg.header.stamp.to_nsec(),
                           right_msg.header.stamp.to_nsec()))
    left_msg.header.stamp = new_stamp
    right_msg.header.stamp = new_stamp

    return left_msg, right_msg


def sync_bag(left_msg, right_msg, bag, topics):
    new_msgs = sync(left_msg, right_msg)
    for topic, msg in zip(topics, new_msgs):
        bag.write(topic, msg, msg.header.stamp)


def topic_names(left, right, raw):
    image_suffix = "/image_raw" + ("" if raw else "/compressed")

    return [left + image_suffix, right + image_suffix]


def bag_main(args):

    rospy.init_node("stereo_sync")

    topics = topic_names(args.left, args.right, args.raw)

    print(topics)

    filters = [message_filters.SimpleFilter() for _ in topics]
    sync_filter = message_filters.ApproximateTimeSynchronizer(filters, args.cache_size, args.slop)

    with rosbag.Bag(args.ibag, 'r') as ibag, rosbag.Bag(args.obag, 'w') as obag:
        sync_filter.registerCallback(sync_bag, obag, topics)
        for topic, msg, t in tqdm(ibag.read_messages(), total=ibag.get_message_count()):
            if topic in topics:
                filters[topics.index(topic)].signalMessage(msg)
            # else:
            #     obag.write(topic, msg, t)


def main():
    argp = argparse.ArgumentParser(
        "Uses ApproximateTimeSynchronizer to force synchronized stereo images")
    argp.add_argument("--ibag", nargs="?", metavar="IN",
                      help="Path to input rosbag file (run live node if empty)")
    argp.add_argument("--obag",  nargs="?", metavar="OUT",
                      help="Path to output rosbag file (run live node if empty)")
    argp.add_argument("--left", default="/cam_fl", help="Left topic")
    argp.add_argument("--right", default="/cam_fr", help="Right topic")
    argp.add_argument("--raw", action="store_true", help="Do not use compressed topics")
    argp.add_argument("--cache_size", default=100, type=int, help="Cache size")
    argp.add_argument("--slop", default=0.02, type=float, help="Maximum deviation between messages")
    args = argp.parse_args(rospy.myargv()[1:])

    # if args.ibag is None and args.obag is None:
    #     node_main(args)
    # elif args.ibag is None or args.obag is None:
    #     argp.error("Both IN and OUT must be specified to process rosbags")
    # else:
    bag_main(args)


if __name__ == "__main__":
    main()
