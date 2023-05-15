#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
import cv2
import json
import os

KITTI = "kitti"
WEIYA = "weiya"


### KITTI specific functionality

def get_K_kitti(projection):
    K = projection[1:4] + projection[5:8] + projection[9:12]
    K = [float(x) for x in K]
    b = [float(projection[4]), float(projection[8]), float(projection[12])]
    return K, b


def read_intrinsics_kitti(args):
    with open(args.in_file_intrinsics, 'r') as input_file:
        content = [line.split() for line in input_file]
        p0 = content[0]
        p1 = content[1]
        assert p0[0] == "P0:"
        assert p1[0] == "P1:"

        K, b = get_K_kitti(p0)
        assert b == [0.0, 0.0, 0.0]  # No translation on left eye
        K_p1, b = get_K_kitti(p1)
        assert K == K_p1  # Same camera matrix for left and right eye

        return K, b


### Weiya specific functionality

def get_K_weiya(projection):
    K = projection[:, 0:3]
    K = [float(k) for row in K for k in row]

    # FIXME(MvB): Values might not be usable yet (but they are also not used at the moment)
    # More information on the functions likely to be used, see function
    # StereoRectify at https://docs.opencv.org/2.4.13.7/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    b = projection[:, 3]
    b = [float(x) for x in b]

    return K, b


def read_intrinsics_weiya(args):
    fs = cv2.FileStorage(args.in_file_intrinsics, cv2.FILE_STORAGE_READ)
    p0 = fs.getNode("P1").mat()  # Left camera
    p1 = fs.getNode("P2").mat()  # Right camera

    K, b = get_K_weiya(p0)
    assert b == [0.0, 0.0, 0.0]  # No translation on left eye
    K_p1, b = get_K_weiya(p1)
    assert K == K_p1  # Same camera matrix for left and right eye

    return K, b


### Generic functionality

def read_image_size(root, file):
    image_file = os.path.join(root, file)
    image = cv2.imread(image_file)
    return image.shape[0], image.shape[1]  # rows, cols


def determine_image_size(args):
    rows = 0
    cols = 0
    for root, _, files in os.walk(args.in_dir_images):
        assert files, f"No images found in path '{args.in_dir_images}'"
        rows, cols = read_image_size(root, files[0])

        if not args.check_image_sizes:
            break

        for file in files[1:]:
            other_rows, other_cols = read_image_size(root, file)
            assert [other_rows, other_cols] == [rows,
                                                cols], f"Image not of same dimensions: '{file}'"

    return rows, cols


def convert(args):
    if args.format == KITTI:
        K, baseline = read_intrinsics_kitti(args)
    elif args.format == WEIYA:
        K, baseline = read_intrinsics_weiya(args)
    height, width = determine_image_size(args)

    intrinsics_content = {
        "platforms": [
            {
                "id": args.format,
                "image_size": [width, height],
                "K": K,
                "frames": [
                    {
                        "id": "left",
                        "baseline": {
                            "translation": [0.0, 0.0, 0.0],
                            "rotation": [0.0, 0.0, 0.0, 1.0]
                        }
                    }, {
                        "id": "right",
                        "baseline": {
                            "translation": baseline,
                            "rotation": [0.0, 0.0, 0.0, 1.0]
                        }
                    }
                ]
            }
        ]
    }
    with open(args.out_file_intrinsics, 'w') as output_file:
        json.dump(intrinsics_content, output_file, indent=4)


def main():
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_intrinsics", type=str, required=True,
                        help="Path to a input intrinsics file.")
    parser.add_argument("--in_dir_images", type=str, required=True,
                        help="Path to directory with images.")
    parser.add_argument("--format", type=str, required=True,
                        choices=[KITTI, WEIYA],
                        help="Data format to be converted.")
    parser.add_argument("--out_file_intrinsics", type=str, required=True,
                        help="Path to a output intrinsics file.")
    parser.add_argument("--check_image_sizes", action='store_true',
                        help="Flag whether image size consistency should be checked.")
    args = parser.parse_args()

    # Check validity of parameters
    if not os.path.isfile(args.in_file_intrinsics):
        print("ERROR: Specified input intrinsics file can not be found.")
        exit(1)
    if not os.path.isdir(args.in_dir_images):
        print("ERROR: Specified input directory with images can not be found.")
        exit(1)

    # Actually perform conversion
    convert(args)


if __name__ == '__main__':
    main()
