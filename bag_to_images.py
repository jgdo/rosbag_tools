#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def convert_bag_file(bag_file, output_dir, image_topic, safe_prefix=''):
    """Extract a folder of images from a rosbag.
        """

    print "Extract images from %s on topic %s into %s" % (bag_file,
                                                          image_topic, output_dir)

    bag = rosbag.Bag(bag_file, "r")

    os.makedirs(output_dir)

    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        t = rospy.Time(msg.header.stamp.to_nsec())
        # print("{}.{}".format(msg.header.stamp.secs, msg.header.stamp.nsecs))
        # print(msg.header.stamp.to_nsec())
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        file_name = "{}frame_{:05d}_{}.jpg".format(safe_prefix, count, msg.header.stamp.to_nsec())

        cv2.imwrite(os.path.join(output_dir, file_name), cv_img)
        # print "Wrote image %i" % count

        count += 1

    bag.close()

    print("Wrote {} images".format(count))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("safe_prefix", help="Prefix of saved file", default='', nargs='?')

    args = parser.parse_args()

    convert_bag_file(args.bag_file, args.output_dir, args.image_topic, args.safe_prefix)
