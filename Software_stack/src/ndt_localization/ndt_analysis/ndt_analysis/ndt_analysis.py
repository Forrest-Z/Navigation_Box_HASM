#!/usr/bin/env python3

# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

""" Perform analysis for NDT Matching Statistics

This script implements real-time visualization of the NDT Statistics such as:
- Vehicle velocity and yaw rate
- Processing times
- Fitness Score

It does this by subscribing to the ndt_stat topic and then visualizing using matplotlib.

It is meant to be ran as a standalone application:
Example:
        $ python ndt_analysis.py
"""

import argparse
import csv
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
import statistics
from tabulate import tabulate
import threading
import time

from ndt_msgs.msg import NDTStat


class NdtAnalyzer:

    def __init__(self, args):
        # Process input arguments
        self.window = args.window
        self.update_freq = 10
        self.last_x = int(self.window * self.update_freq)
        self.only_stats = args.only_stats
        self.stat_file = args.output_filename + ".csv"
        self.plot_file = args.output_filename + ".png"
        self.no_filesave = args.no_filesave

        # Set initial values
        self.initialized = False
        self.time_abs = 0.0

        # Create data storage lists
        self.msg_time = []
        self.velocity = []
        self.yaw_rate = []
        self.response_time = []
        self.latency = []
        self.processing_time = []
        self.align_time = []
        self.fitness_time = []
        self.fitness_score = []
        self.tf_probability = []

        if not self.only_stats:
            self.initialize_figure()

        # Create mutex for safe data handling
        self.mutex = threading.Lock()

        # Subscribe to statistics topic
        rclpy.Subscriber("/ndt_stat", NDTStat, self.ndt_stat_callback)

        # Set time to start logging
        self.time_start = rclpy.get_rostime()

    def initialize_figure(self):
        """ Initializes the figure for plotting data """

        # Define limits
        self.lim_vel = [-1.0, 8.0]
        self.lim_yaw = [-0.7, 0.7]
        self.lim_time = [0.5, 110.0]
        self.lim_score = [0.01, 1000.0]
        self.lim_tf = [0.0, 2.5]

        # Define colors
        self.color_vel = 'tab:red'
        self.color_yaw = 'tab:blue'
        self.color_response = 'tab:gray'
        self.color_latency = 'tab:purple'
        self.color_processing = 'tab:green'
        self.color_align = 'tab:blue'
        self.color_fitness = 'tab:red'
        self.color_score = 'tab:red'
        self.color_tf = 'tab:blue'

        # Enable interactive plotting mode.
        # This is required for dynamically updating the plots
        plt.ion()

        # Create figure
        self.fig, (self.ax_vel, self.ax_time, self.ax_score) = plt.subplots(
            3, 1, figsize=(16, 8), dpi=100)
        # Instantiate a second axis that shares the same x-axis
        self.ax_yaw = self.ax_vel.twinx()
        self.ax_tf = self.ax_score.twinx()

        # Create and assign plots
        self.line_vel, = self.ax_vel.plot(
            self.msg_time, self.velocity, color=self.color_vel)
        self.line_yaw, = self.ax_yaw.plot(
            self.msg_time, self.yaw_rate, color=self.color_yaw)
        self.line_response_time, = self.ax_time.plot(
            self.msg_time, self.response_time, color=self.color_response, label='Response')
        self.line_latency, = self.ax_time.plot(
            self.msg_time, self.latency, color=self.color_latency, label='Latency')
        self.line_processing_time, = self.ax_time.plot(
            self.msg_time, self.processing_time, color=self.color_processing, label='Processing')
        self.line_align_time, = self.ax_time.plot(
            self.msg_time, self.align_time, color=self.color_align, label='Align')
        self.line_fitness_time, = self.ax_time.plot(
            self.msg_time, self.fitness_time, color=self.color_fitness, label='Fitness')
        self.line_score, = self.ax_score.plot(
            self.msg_time, self.fitness_score, color=self.color_score)
        self.line_tf_probability, = self.ax_tf.plot(
            self.msg_time, self.tf_probability, color=self.color_tf)

        # Setup figure layout
        # Set x labels
        self.ax_score.set_xlabel('time [s]')
        # Set y labels
        self.ax_vel.set_ylabel('velocity [m/s]', color=self.color_vel)
        self.ax_yaw.set_ylabel('yaw rate [rad/s]', color=self.color_yaw)
        self.ax_time.set_ylabel('processing time [ms]')
        self.ax_time.set_yscale('log')
        self.ax_score.set_ylabel('fitness score [-]', color=self.color_score)
        self.ax_score.set_yscale('log')
        self.ax_tf.set_ylabel('tf probability [-]', color=self.color_tf)
        self.ax_tf.set_yscale('linear')
        # Color y axis for double plot
        self.ax_vel.tick_params(axis='y', labelcolor=self.color_vel)
        self.ax_yaw.tick_params(axis='y', labelcolor=self.color_yaw)
        self.ax_score.tick_params(axis='y', labelcolor=self.color_score)
        self.ax_tf.tick_params(axis='y', labelcolor=self.color_tf)
        # Set y limits
        self.ax_vel.set_ylim(self.lim_vel)
        self.ax_yaw.set_ylim(self.lim_yaw)
        self.ax_time.set_ylim(self.lim_time)
        self.ax_score.set_ylim(self.lim_score)
        self.ax_tf.set_ylim(self.lim_tf)
        # Set legend
        self.ax_time.legend(loc=3, ncol=5, fontsize='small')
        # Set grid
        self.ax_vel.grid()
        self.ax_time.grid(b=True, which='both')
        self.ax_score.grid(b=True, which='both')
        # Fix double y axis clipping
        self.fig.tight_layout()

    def get_single_statistics(self, data, name):
        """ Get mean, stdev, variance, and min max values for the given data set """
        return [
            name,
            statistics.mean(data),
            statistics.stdev(data),
            statistics.variance(data),
            min(data),
            max(data)]

    def print_statistics(self):
        """ Print the statistics in a table """
        if len(self.processing_time) <= 1:
            return
        stat_data = [
            self.get_single_statistics(
                self.processing_time, "Processing"),
            self.get_single_statistics(
                self.align_time, "Align time"),
            self.get_single_statistics(
                self.fitness_time, "Fitness time"),
            self.get_single_statistics(
                self.fitness_score, "Fitness Score"),
            self.get_single_statistics(
                self.tf_probability, "Tf probability")]
        header_row = ["Stat", "Mean", "Stddev", "Variance", "Min", "Max"]
        print()
        print(
            tabulate(
                stat_data,
                headers=header_row,
                tablefmt="fancy_grid",
                floatfmt=".4f"))
        # We overwrite the file every time as we are only interested in the
        # total set of data
        if not self.no_filesave:
            with open(self.stat_file, "w") as f:
                writer = csv.writer(f)
                writer.writerow(header_row)
                writer.writerows(stat_data)

    def ndt_stat_callback(self, ndt_stat):
        """ This callback saves the ndt_stat data into the data arrays """
        if not self.initialized:
            self.time_start = ndt_stat.header.stamp
            self.initialized = True
        self.time_abs = (ndt_stat.header.stamp - self.time_start).to_sec()

        self.mutex.acquire()
        self.msg_time.append(self.time_abs)
        self.velocity.append(ndt_stat.velocity)
        self.yaw_rate.append(ndt_stat.yaw_rate)
        self.response_time.append(ndt_stat.response_time)
        self.latency.append(ndt_stat.latency)
        self.processing_time.append(ndt_stat.processing_time)
        self.align_time.append(ndt_stat.align_time)
        self.fitness_time.append(ndt_stat.fitness_time)
        self.fitness_score.append(ndt_stat.fitness_score)
        self.tf_probability.append(ndt_stat.tf_probability)
        self.mutex.release()

        self.print_statistics()

    def run(self):
        """ This simply keeps the node/thread running until stopped """
        rclpy.spin()

    def update_line(self, line, data):
        """ Updates the given plot line with the latest data """
        line.set_xdata(self.msg_time[-self.last_x:])
        line.set_ydata(data[-self.last_x:])

    def resize_x_axis(self, axis):
        """ Resizes the x axis to match the latest data """
        axis.relim()
        axis.autoscale_view()

    def update_plot(self):
        """ Updates the figure to display the latest data """

        # Check if the latest data has finished processing
        self.mutex.acquire()
        # Update the line data
        self.update_line(self.line_vel, self.velocity)
        self.update_line(self.line_yaw, self.yaw_rate)
        self.update_line(self.line_response_time, self.response_time)
        self.update_line(self.line_latency, self.latency)
        self.update_line(self.line_processing_time, self.processing_time)
        self.update_line(self.line_align_time, self.align_time)
        self.update_line(self.line_fitness_time, self.fitness_time)
        self.update_line(self.line_score, self.fitness_score)
        self.update_line(self.line_tf_probability, self.tf_probability)
        self.mutex.release()

        # Resize x axes
        self.resize_x_axis(self.ax_vel)
        self.resize_x_axis(self.ax_time)
        self.resize_x_axis(self.ax_score)

        # Update the figure
        self.fig.canvas.draw()
        # Save to file
        if not self.no_filesave:
            self.fig.savefig(self.plot_file, bbox_inches='tight')


if __name__ == '__main__':
    # Set up parser
    analysis_parser = argparse.ArgumentParser()
    analysis_parser.add_argument(
        '--window',
        dest='window',
        type=float,
        default=0.0,
        help="Only show last WINDOW seconds")
    analysis_parser.add_argument(
        '--only_stats',
        action='store_true',
        dest='only_stats',
        help="Show data plot")
    analysis_parser.add_argument(
        '--output',
        dest='output_filename',
        default="./stat_analysis",
        help='File path to store statistics')
    analysis_parser.add_argument(
        '--no_filesave',
        action='store_true',
        dest='no_filesave',
        help="Don't save any files")

    args, _ = analysis_parser.parse_known_args()

    rclpy.init_node('ndt_analysis', anonymous=True)

    ndt_analysis = NdtAnalyzer(args)

    if args.only_stats:
        ndt_analysis.run()
    else:
        # If we need to plot, we need to update it in the main thread
        # This because tkinter is not threadsafe
        t_ros = threading.Thread(target=ndt_analysis.run)
        t_ros.start()

        loop_interval = 0.1
        # Start visualization loop
        while t_ros.is_alive():
            loop_start = time.time()

            ndt_analysis.update_plot()

            # Sleep for next update interval
            dt = time.time() - loop_start
            if loop_interval - dt > 0.0:
                time.sleep(loop_interval - dt)

        # Close monitors
        t_ros.join()
