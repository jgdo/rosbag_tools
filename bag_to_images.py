#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse
from tqdm import tqdm

import cv2

import rosbag
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np

def convert_bag_file(bag_file, output_dir, image_topic, safe_prefix='', skip_factor=1):
    """Extract a folder of images from a rosbag.
        """

    print "Extract images from %s on topic %s into %s" % (bag_file,
                                                          image_topic, output_dir)

    bag = rosbag.Bag(bag_file, "r")

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    bridge = CvBridge()
    count = 0
    total_count = 0
    for topic, msg, t in tqdm(bag.read_messages(topics=[image_topic])):
        if total_count % skip_factor == 0:
            t = rospy.Time(msg.header.stamp.to_nsec())
            # print("{}.{}".format(msg.header.stamp.secs, msg.header.stamp.nsecs))
            # print(msg.header.stamp.to_nsec())

            if topic.endswith("compressed"):
                np_arr = np.fromstring(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

            file_name = "{}frame_{:05d}_{}.jpg".format(safe_prefix, count, msg.header.stamp.to_nsec())

            cv2.imwrite(os.path.join(output_dir, file_name), cv_img)
            # print "Wrote image %i" % count

            count += 1

        total_count += 1

    bag.close()

    print("Wrote {} images".format(count))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("-s", "--skip", help="Skip factor", default=1, type=int)
    parser.add_argument("safe_prefix", help="Prefix of saved file", default='', nargs='?')

    args = parser.parse_args()

    convert_bag_file(args.bag_file, args.output_dir, args.image_topic, args.safe_prefix, args.skip)
