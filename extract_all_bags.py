import os
import argparse

from bag_to_images import convert_bag_file

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Extract images from all ros bags")
    parser.add_argument("bag_dir", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")

    args = parser.parse_args()

    bag_files = [f for f in os.listdir(args.bag_dir) if os.path.isfile(os.path.join(args.bag_dir, f)) and f.endswith('.bag')]

    topics_dict = {'cam_1': '/cam_1/color/image_raw', 'cam_3': '/cam_3/color/image_raw'}

    for bf in bag_files:
        print("Reading {}".format(bf))

        bf_base = os.path.splitext(bf)[0]
        images_output_base_dir = os.path.join(args.output_dir, bf_base)

        bf_path = os.path.join(args.bag_dir, bf)

        for prefix, topic in topics_dict.items():
            # print('Extracting {}'.format(topic))
            images_output_dir = os.path.join(images_output_base_dir, prefix)
            convert_bag_file(bf_path, images_output_dir, topic)
