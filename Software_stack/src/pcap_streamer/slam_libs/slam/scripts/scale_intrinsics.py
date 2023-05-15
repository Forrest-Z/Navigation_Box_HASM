#!/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import json
import argparse


# This script scales the focal length, principle point and baseline translation
# of the given intrinsics file.


def scale_platform(platform, scale):
    # In slam_tue the image resize function is used, which rounds the image
    # sizes, therefore the same is applied here
    platform["image_size"] = [round(x * scale) for x in platform["image_size"]]

    platform["K"] = [x * scale for x in platform["K"]]
    platform["K"][-1] = 1.0

    return platform


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file", type=str,
                        help="Path to input intrinsics json file.")
    parser.add_argument("--out_file", type=str,
                        help="Path to output intrinsics json file.")
    parser.add_argument("--scale", type=float, help="Multiplication scalar.")
    args = parser.parse_args()

    data_in = None
    with open(args.in_file, 'r') as in_file:
        data_in = json.load(in_file)
        print(f"Read input file: {args.in_file}")

    if data_in:
        data_in["platforms"] = \
            [scale_platform(p, args.scale) for p in data_in["platforms"]]

        with open(args.out_file, 'w') as out_file:
            json.dump(data_in, out_file, indent=4)

        print(f"Written scaled intrinsics to file: {args.out_file}")
    else:
        print(f"Could not input read file: {args.in_file}")
