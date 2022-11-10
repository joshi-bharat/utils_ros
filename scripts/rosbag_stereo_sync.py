#! /usr/bin/env python

import rosbag
import rospy
import numpy as np
import message_filters
from tqdm import tqdm
import argparse

left_stamps = []
right_stamps = []
common_stamps = []
stamps = open("stamps.txt", "w")
stamps.write("Left, Right, Common\n")


def topic_names(left, right, raw):
    image_suffix = "/image_raw" + ("" if raw else "/compressed")

    return [left + image_suffix, right + image_suffix]


def stereo_callback(left_msg, right_msg, output_bag, topics):
    # Matches timestamps in left and right messages
    new_nsecs = int(
        np.double(left_msg.header.stamp.to_nsec() + right_msg.header.stamp.to_nsec())
        / 2.0
    )
    new_stamp = rospy.Time(nsecs=new_nsecs)

    # writing for post processing if needed
    stamps.write(
        f"{str(left_msg.header.stamp.to_nsec())},{str(right_msg.header.stamp.to_nsec())},{new_stamp.to_nsec()}\n"
    )

    left_stamps.append(left_msg.header.stamp.to_nsec())
    right_stamps.append(right_msg.header.stamp.to_nsec())
    common_stamps.append(new_stamp.to_nsec())

    left_msg.header.stamp = new_stamp
    right_msg.header.stamp = new_stamp

    output_bag.write(topics[0], left_msg, new_stamp)
    output_bag.write(topics[1], right_msg, new_stamp)


def sync_bag(args):

    rospy.init_node("stereo_sync")

    topics = topic_names(args.left, args.right, args.raw)

    filters = [message_filters.SimpleFilter() for _ in topics]
    sync_filter = message_filters.ApproximateTimeSynchronizer(
        filters, args.cache_size, args.slop
    )
    outbag = rosbag.Bag(args.obag, "w")
    sync_filter.registerCallback(stereo_callback, outbag, topics)

    with rosbag.Bag(args.ibag, "r") as ibag:
        for topic, msg, t in tqdm(ibag.read_messages(), total=ibag.get_message_count()):  # type: ignore
            if topic in topics:
                filters[topics.index(topic)].signalMessage(msg)
            else:
                outbag.write(topic, msg, t)

    stamps.close()
    outbag.close()


if __name__ == "__main__":
    argp = argparse.ArgumentParser(
        "Uses ApproximateTimeSynchronizer to force synchronized stereo images"
    )
    argp.add_argument(
        "--ibag",
        nargs="?",
        metavar="IN",
        help="Path to input rosbag file (run live node if empty)",
    )
    argp.add_argument(
        "--obag",
        nargs="?",
        metavar="OUT",
        help="Path to output rosbag file (run live node if empty)",
    )
    argp.add_argument("--left", default="/slave1", help="Left topic")
    argp.add_argument("--right", default="/slave2", help="Right topic")
    argp.add_argument(
        "--raw", action="store_true", help="Do not use compressed topics", default=False
    )
    argp.add_argument("--cache_size", default=100, type=int, help="Cache size")
    argp.add_argument(
        "--slop", default=0.01, type=float, help="Maximum deviation between messages"
    )
    args = argp.parse_args(rospy.myargv()[1:])

    sync_bag(args)
