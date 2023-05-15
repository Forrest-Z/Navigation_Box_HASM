#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
"""Statistics plotter

This script receives a JSON file containing extended calibration data from the calibrator application and generates
various plots based on user choice.

Examples:
    Generate residuals scatter plot:
        $ python3 statistics_plotter.py -p /path/to/file.json
    Generate and show residuals scatter plot:
        $ python3 statistics_plotter.py -s -p /path/to/file.json
    More information:
        $ python3 statistics_plotter.py -h
"""


import argparse
import json
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import itertools


DEFAULT_IMAGE_SOURCES = ('extended_data', 'extended_data_left', 'extended_data_right', 'validated_data_left',
                         'validated_data_right', 'validated_data')
STD_DEV_THRESHOLD_FACTOR = 3.0
RESIDUAL_SCATTER_PLOT_PADDING_PERCENTAGE = 0.5
# pyplot can save figures in both png and pdf formats
DEFAULT_OUTPUT_FILE_EXTENSION = "png"
DEFAULT_SCATTER_PLOT_FILENAME = "scatter_plot"
DEFAULT_CHECKERBOARD_SCATTER_PLOT_FILENAME = "checkerboard_scatter_plot"
DEFAULT_FEATURE_SCATTER_PLOT_FILENAME = "feature_scatter_plot"
DEFAULT_TILE_SCATTER_PLOT_FILENAME = "tile_scatter_plot"
DEFAULT_EPIPOLAR_ERROR_HISTOGREAM_FILENAME = "epipolar_error_histogram"
DEFAULT_TILE_SIZE = 64
VALID_DATA_TYPES = ('residuals', 'image_points')
DEFAULT_HISTOGRAM_NUMBER_OF_BINS = 1000


def parse_arguments():
    """
    Parse input arguments
    :return: parsed arguments
    """
    parser = argparse.ArgumentParser(description='Plot various plots of the residuals that come from an extended'
                                                 'calibration data JSON file')
    # Positional args
    parser.add_argument("extended_calibration_data_path", type=str,
                        help="path to a JSON file containing extended calibration data")
    # Optional args
    parser.add_argument("-s", "--show-plots", action='store_true', help="show interactive plots")

    parser.add_argument("-p", "--scatter-plot", action='store_true', help="generate residuals scatter plot")
    parser.add_argument("-pp", "--scatter-plot-path", type=str,
                        default=DEFAULT_SCATTER_PLOT_FILENAME + '.' + DEFAULT_OUTPUT_FILE_EXTENSION,
                        help="scatter plot output path")

    parser.add_argument("-c", "--checkerboard-scatter-plot", action='store_true',
                        help="generate residuals scatter plot on a virtual checkerboard in a per-corner manner")
    parser.add_argument("-cp", "--checkerboard-scatter-plot-path", type=str,
                        default=DEFAULT_CHECKERBOARD_SCATTER_PLOT_FILENAME + '.' + DEFAULT_OUTPUT_FILE_EXTENSION,
                        help="checkerboard scatter plot output path")

    parser.add_argument("-f", "--feature-scatter-plot", action='store_true',
                        help="generate feature scatter plot")
    parser.add_argument("-fp", "--feature-scatter-plot-path", type=str,
                        default=DEFAULT_FEATURE_SCATTER_PLOT_FILENAME + '.' + DEFAULT_OUTPUT_FILE_EXTENSION,
                        help="feature scatter plot output path")

    parser.add_argument("-t", "--tile-scatter-plot", action='store_true',
                        help="generate tile scatter plot")
    parser.add_argument("-tp", "--tile-scatter-plot-path", type=str,
                        default=DEFAULT_TILE_SCATTER_PLOT_FILENAME + '.' + DEFAULT_OUTPUT_FILE_EXTENSION,
                        help="tile scatter plot output path")

    parser.add_argument("-e", "--epipolar-error-histogram", action='store_true',
                        help="generate epipolar error histogram")
    parser.add_argument("-ep", "--epipolar-error-histogram-path", type=str,
                        default=DEFAULT_EPIPOLAR_ERROR_HISTOGREAM_FILENAME + '.' + DEFAULT_OUTPUT_FILE_EXTENSION,
                        help="epipolar error histogram output path")

    args = parser.parse_args()
    return args


def parse_calibration_json(filename: str) -> dict:
    """
    Read and parse a JSON file into a dict
    :param filename: path to a JSON file
    :return: parsed dict
    """
    try:
        with open(filename) as f:
            data = json.load(f)
        return data
    except IOError:
        print("ERROR: could not open file {}".format(filename))
        exit(1)


def concatenate_lists(extended_calib_data: dict,
                      data_type: str,
                      source_list=DEFAULT_IMAGE_SOURCES) -> (list, list):
    """
    Concatenate the residuals from a given lists of sources (mono, stereo, etc) into a list of x coordinates and a list
    of y coordinates.
    :param extended_calib_data: list of image data
    :param data_type: 'residuals' or 'image_points'
    :param source_list: list of image sources to be concatenated. Default is all of the possible sources.
    :return: concatenated lists
    """
    # Concatenate the lists
    if data_type not in VALID_DATA_TYPES:
        print("Invalid data type '{}'. Choose between {}. Exiting.".format(data_type, VALID_DATA_TYPES))
        exit(1)
    residuals = []
    for img_source in source_list:
        if img_source in extended_calib_data:
            for img in extended_calib_data[img_source][data_type]:
                residuals += img
    x, y = get_1d_lists_from_2dpoints_list(residuals)
    return x, y


def residual_scatter_plot(extended_calib_data: dict,
                          output_path: str,
                          limit_draw_to_threshold=False):
    """
    Scatter plot of the residuals
    :param extended_calib_data: parsed extended calibration data
    :param output_path: output file path. Can be saved as png or pdf depending of the file extension.
    :param limit_draw_to_threshold: constrain the plot to a threshold that is proportional to the standard deviation.
                                    Defaults to False
    :return:
    """
    std_dev = get_stddev(extended_calib_data)
    # Threshold is calculated w.r.t. std_dev
    threshold = STD_DEV_THRESHOLD_FACTOR * std_dev

    # pyplot setup
    plt.figure(num=pathlib.PurePath(output_path).stem)
    num_points = get_num_of_points(extended_calib_data)
    plt.title("Reprojection Errors for {} points\nstddev = {:.2f} pixels.\nThreshold at {:.2f} pixels".format(
        num_points, std_dev, threshold))
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.grid(linestyle=':')

    # Automatically adjust ranges of the plot to the threshold
    if limit_draw_to_threshold:
        padded_threshold = threshold * (1.0 + RESIDUAL_SCATTER_PLOT_PADDING_PERCENTAGE)
        draw_limits = (-padded_threshold, padded_threshold)
        plt.xlim(draw_limits)
        plt.ylim(draw_limits)

    # Plot stddev
    std_dev_circle = plt.Circle((0, 0), std_dev, fill=False)
    plt.gcf().gca().add_artist(std_dev_circle)

    # Plot threshold (STD_DEV_THRESHOLD_FACTOR * std_dev)
    threshold_circle = plt.Circle((0, 0), threshold, fill=False, linestyle='--')
    plt.gcf().gca().add_artist(threshold_circle)

    plt.gca().invert_yaxis()
    # Making sure the scales are the same for both axes
    plt.axes().set_aspect('equal', 'box')

    # Organize the data
    x, y = concatenate_lists(extended_calib_data, 'residuals')
    plt.scatter(x, y, s=1.0)
    plt.savefig(output_path, bbox_inches='tight')


def get_2d_index_from_1d_index(pattern_size: list,
                               flat_index: int) -> (int, int):
    """
    Obtain the 2D index from an 1D index given a pattern size. This can be thought as mapping a flat vector to a 2D
    matrix.
    :param pattern_size: size of the pattern
    :param flat_index: input 1D index
    :return: 2D index
    """
    y = flat_index // pattern_size[0]
    x = flat_index % pattern_size[0]
    return y, x


def translate_points(points: list,
                     pattern_size: list,
                     offset: float) -> list:
    """
    Translate all the points in a flat list by an offset.
    :param points: 1D list of points
    :param pattern_size: 2D size of the pattern,
    :param offset: translation offset
    :return: translated list of points
    """
    temp_list = []
    for index in range(len(points)):
        j, i = get_2d_index_from_1d_index(pattern_size, index)
        el = points[index]
        t_x = el[0] + i * offset
        t_y = el[1] + j * offset
        temp_list.append([t_x, t_y])
    return temp_list


def get_num_of_imgs(extended_calib_data: dict,
                    sources_list: list = DEFAULT_IMAGE_SOURCES) -> int:
    """
    Get the number of images from a list of image sources
    :param extended_calib_data: parsed extended calibration data
    :param sources_list: image sources
    :return:
    """
    num = 0
    for source in sources_list:
        if source in extended_calib_data:
            num += len(extended_calib_data[source]['residuals'])
    return num


def get_num_of_points(extended_calib_data: dict,
                      sources_list: list = DEFAULT_IMAGE_SOURCES) -> int:
    """
    Get the number of points from a list of image sources
    :param extended_calib_data: parsed extended calibration data
    :param sources_list: image sources
    :return:
    """
    if 'pattern_size' not in extended_calib_data:
        return 0
    num_imgs = get_num_of_imgs(extended_calib_data, sources_list)
    pattern_size = extended_calib_data['pattern_size']
    return num_imgs * pattern_size[0] * pattern_size[1]


def get_stddev(extended_calib_data):
    x, y = concatenate_lists(extended_calib_data, 'residuals')
    std_dev_str = "stddev_residuals"
    if std_dev_str not in extended_calib_data:
        std_dev_x = np.array(x).std()
        std_dev_y = np.array(y).std()
        return np.sqrt(std_dev_x ** 2 + std_dev_y ** 2)
    else:
        return extended_calib_data[std_dev_str]


def checkerboard_residual_scatter_plot(extended_calib_data: dict,
                                       output_path: str):
    """
    Plot the residuals on a "virtual" checkerboard
    :param extended_calib_data: parsed extended calibration data
    :param output_path: output file path. Can be saved as png or pdf depending of the file extension.
    :return:
    """
    pattern_size = []
    pattern_size_str = 'pattern_size'
    if pattern_size_str in extended_calib_data:
        pattern_size = extended_calib_data['pattern_size']
    else:
        print("'{} field not found. Aborting checkerboard plot.'".format(pattern_size_str))
        return
    num_points_per_img = pattern_size[0] * pattern_size[1]
    x, y = concatenate_lists(extended_calib_data, 'residuals')
    combined_list = list(zip(x, y))
    combined_list = np.array(combined_list).reshape(len(combined_list) // num_points_per_img, num_points_per_img, 2)

    std_dev = get_stddev(extended_calib_data)
    # Placing each corner plot in a manner that allows enough space for *most* of the points to be overlap-free with
    # points from different corners.
    tile_size = 2 * STD_DEV_THRESHOLD_FACTOR * std_dev

    # Pyplot setup
    plt.figure(num=pathlib.PurePath(output_path).stem)
    num_points = get_num_of_points(extended_calib_data)
    plt.title("Reprojection Errors distributed around checkerboard corners. {} points\n Global stddev = {:.2f} pixels."
              .format(num_points, std_dev))
    plt.xlabel("Corner X index")
    plt.ylabel("Corner Y index")
    plt.gca().invert_yaxis()
    plt.grid(linestyle=':')
    # Making sure the scales are the same for both axes
    plt.axes().set_aspect('equal', 'box')
    x_range = range(pattern_size[0])
    y_range = range(pattern_size[1])
    # Bind the ticks to the indices of the corners, not to the position of the residuals
    plt.xticks([val * tile_size for val in x_range], x_range)
    plt.yticks([val * tile_size for val in y_range], y_range)
    # Translating the residuals in order to create a "virtual" checkerboard pattern
    translated_residuals = [translate_points(res, pattern_size, tile_size) for res in combined_list]
    # Converting to np.array makes slicing straight-forward
    translated_residuals = np.array(translated_residuals)
    # Switch 'images' with 'corners'
    corners = translated_residuals.transpose((1, 0, 2))
    # Interpolate colors in the jet colormap
    colors = cm.jet(np.linspace(0, 1, len(corners)))
    for corner, color in zip(corners, colors):
        x = corner[:, 0]
        y = corner[:, 1]
        plt.scatter(x, y, s=1.0, c=[color])
    plt.savefig(output_path, bbox_inches='tight')


def feature_scatter_plot(extended_calib_data: dict,
                         output_path: str,
                         image_size: list):
    """
    Plot the coordinates of the detected features
    :param extended_calib_data: parsed extended calibration data
    :param output_path: output file path. Can be saved as png or pdf depending of the file extension.
    :param image_size: list containing the width and height of the image
    :return:
    """
    if not image_size:
        print("Image size not available. Aborting feature scatter plot.")
        return
    # Pyplot setup
    plt.figure(num=pathlib.PurePath(output_path).stem)
    # Making sure the scales are the same for both axes
    plt.axes().set_aspect('equal', 'box')
    num_points = get_num_of_points(extended_calib_data)
    plt.title("Feature (corner) coordinates. {} points".format(num_points))
    plt.xlabel("X index")
    plt.ylabel("Y index")
    plt.grid(linestyle=':')
    plt.xlim(0, image_size[0])
    plt.ylim(0, image_size[1])
    plt.gca().invert_yaxis()

    x, y = concatenate_lists(extended_calib_data, 'image_points')
    plt.scatter(x=x, y=y, s=1.0)
    plt.savefig(output_path, bbox_inches='tight')

def get_1d_lists_from_2dpoints_list(list_2d: list) -> (list, list):
    """
    Get 2 1-dimensional lists from a flattened 2d list.
    For instance:
        input:
            list_2d = [(1, 2), (3, 4), (5,6)]
        output:
            ([1, 3, 5], [2, 4, 6])

    :param list_2d:
    :return: 2 1-dimensional lists
    """
    x, y = [i[0] for i in list_2d], [i[1] for i in list_2d]
    assert len(x) == len(y)
    return x, y


def tile_scatter_plot(extended_calib_data: dict,
                      output_path: str,
                      image_size: list,
                      source_list: list = DEFAULT_IMAGE_SOURCES):
    """
    Plot the residual errors per tile.
    :param extended_calib_data: parsed extended calibration data
    :param output_path: output file path. Can be saved as png or pdf depending of the file extension.
    :param image_size: size of the image in pixels.
    :param source_list: list of image sources to be concatenated. Default is all of the possible sources.
    :return:
    """
    if not image_size:
        print("Image size not available. Aborting tile scatter plot.")
        return
    # Pyplot setup
    plt.figure(num=pathlib.PurePath(output_path).stem)
    tile_size_x = DEFAULT_TILE_SIZE
    tile_size_y = DEFAULT_TILE_SIZE

    std_dev = get_stddev(extended_calib_data)
    tile_spacing_y = 2 * STD_DEV_THRESHOLD_FACTOR * std_dev
    tile_spacing_x = tile_spacing_y

    # Adding 1 to account for leftover tiles, i.e., tiles which dimensions are not DEFAULT_TILE_SIZE * DEFAULT_TILE_SIZE
    x_range = range(image_size[0] // DEFAULT_TILE_SIZE + 1)
    y_range = range(image_size[1] // DEFAULT_TILE_SIZE + 1)
    # Bind the ticks to the indices of the tiles, not to the position of the residuals
    plt.xticks([val * tile_spacing_x for val in x_range], x_range)
    plt.yticks([val * tile_spacing_y for val in y_range], y_range)
    plt.xlim(-tile_spacing_x, x_range.stop * tile_spacing_x)
    plt.ylim(-tile_spacing_y, y_range.stop * tile_spacing_y)
    # Making sure the scales are the same for both axes
    plt.axes().set_aspect('equal', 'box')
    num_points = get_num_of_points(extended_calib_data)
    plt.title("Reprojection errors per tile ({} x {} pixels). {} points".format(tile_size_x, tile_size_y, num_points))
    plt.xlabel("Tile X index")
    plt.ylabel("Tile Y index")
    plt.grid(linestyle=':')
    plt.gca().invert_yaxis()

    corners = []
    residuals = []

    # Append elements from different valid sources (e.g. 'extended_data_left' and 'extended_data_right')
    for source in source_list:
        if source in extended_calib_data:
            extended_calib_data_from_source = extended_calib_data[source]
            corners += extended_calib_data_from_source['image_points']
            residuals += extended_calib_data_from_source['residuals']

    # Filter possible empty residuals lists and remove the related list from the list of corners per image
    corners = np.array([np.array(el) for el in corners])
    residuals = np.array([np.array(el) for el in residuals])
    selector = [li.size != 0 for li in residuals]
    filtered_corners = corners[selector]
    filtered_residuals = residuals[selector]

    # Flatten lists
    flat_residuals = list(itertools.chain.from_iterable(filtered_residuals))
    flat_corners = list(itertools.chain.from_iterable(filtered_corners))

    corners_x, corners_y = get_1d_lists_from_2dpoints_list(flat_corners)
    residuals_x, residuals_y = get_1d_lists_from_2dpoints_list(flat_residuals)
    assert len(corners_x) == len(corners_y) == len(residuals_x) == len(residuals_y)

    # Generate a list of lists that will contain the reprojection errors of a given tile. An empty inner list mean that
    # no corner was detected in the related tile.
    tiles = [[[] for __ in x_range] for __ in y_range]
    for i in range(len(corners_x)):
        corner_x = corners_x[i]
        corner_y = corners_y[i]
        residual_x = residuals_x[i]
        residual_y = residuals_y[i]

        # Bin the residuals into the tiles
        x = int(corner_x // tile_size_x)
        y = int(corner_y // tile_size_y)
        tiles[y][x].append([residual_x, residual_y])

    # Interpolate the colors from 0 to the number of tiles
    colors = cm.jet(np.linspace(0, 1, len(tiles) * len(tiles[0])))
    for row in range(len(tiles)):
        for col in range(len(tiles[row])):
            el = tiles[row][col]
            if len(el) != 0:
                el = np.array(el)
                x = el[:, 0]
                y = el[:, 1]
                # Plot the residuals per tile
                plt.scatter(x + col * tile_spacing_x,
                            y + row * tile_spacing_y,
                            s=1.0,
                            c=[colors[row * len(tiles[0]) + col]])
    plt.savefig(output_path, bbox_inches='tight')


def epipolar_scatter_plot(extended_calib_data: dict,
                          output_path: str):
    """
    Plot the histogram of epipolar errors.
    :param extended_calib_data: parsed extended calibration data
    :param output_path: output file path. Can be saved as png or pdf depending of the file extension.
    :return:
    """

    validated_data = 'epipolar_errors'
    if validated_data not in extended_calib_data:
        print("'{}' data is not present. Not generating plot.".format(validated_data))
        return
    # Pyplot setup
    plt.figure(num=pathlib.PurePath(output_path).stem)
    # Flatten the lists
    epipolar_errors = list(itertools.chain.from_iterable(extended_calib_data["epipolar_errors"]))
    num_points = len(epipolar_errors)
    plt.title("Epipolar errors. {} points".format(num_points))
    plt.xlabel("Error")
    plt.ylabel("Count")
    plt.grid(linestyle=':')
    plt.hist(epipolar_errors, DEFAULT_HISTOGRAM_NUMBER_OF_BINS)
    plt.savefig(output_path, bbox_inches='tight')


def no_plot_selected(args) -> bool:
    """
    Check if no plot is selected. This function should be updated when a new plot is added.
    :param args: parsed arguments from argparse
    :return: True if no plot is selected, False otherwise
    """
    return not (args.scatter_plot or
                args.checkerboard_scatter_plot or
                args.feature_scatter_plot or
                args.tile_scatter_plot or
                args.epipolar_error_histogram)


def main():
    args = parse_arguments()
    if no_plot_selected(args):
        print("WARNING: no plot selected, exiting")
        exit(0)

    show_plots = args.show_plots
    generate_scatter_plot = args.scatter_plot
    generate_checkerboard_scatter_plot = args.checkerboard_scatter_plot
    generate_feature_scatter_plot = args.feature_scatter_plot
    generate_tile_scatter_plot = args.tile_scatter_plot
    generate_epipolar_error_histogram = args.epipolar_error_histogram

    extended_calibration_data_path = args.extended_calibration_data_path
    extended_calibration_data = parse_calibration_json(extended_calibration_data_path)

    image_size = []
    if 'image_size' in extended_calibration_data:
        image_size = extended_calibration_data['image_size']

    # At this point *at least* one plot has been selected
    if generate_scatter_plot:
        residual_scatter_plot(extended_calibration_data, args.scatter_plot_path)
    if generate_checkerboard_scatter_plot:
        checkerboard_residual_scatter_plot(extended_calibration_data, args.checkerboard_scatter_plot_path)
    if generate_feature_scatter_plot:
        feature_scatter_plot(extended_calibration_data, args.feature_scatter_plot_path, image_size)
    if generate_tile_scatter_plot:
        tile_scatter_plot(extended_calibration_data,
                          args.tile_scatter_plot_path,
                          image_size)
    if generate_epipolar_error_histogram:
        epipolar_scatter_plot(extended_calibration_data, args.epipolar_error_histogram_path)
    if show_plots:
        plt.show()


if __name__ == "__main__":
    main()
