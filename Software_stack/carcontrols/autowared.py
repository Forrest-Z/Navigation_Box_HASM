#!/usr/bin/env python
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

"""
This process publishes Autoware twist messages over a zmq socket, which are read by autoware_controls
"""
import threading
# Openpilot imports
import zmq
from selfdrive.services import service_list
import selfdrive.messaging as messaging
# Autoware/ROS imports
import rospy
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Bool
from autoware_msgs.msg import ControlCommandStamped
import time


def is_main_thread_active():
    """ Check if the main thread is still active. Return True if it is. False if it isn't"""
    # In Python 3.4+, you can use threading.main_thread().is_alive() instead
    return any((i.name == "MainThread") and i.is_alive()
               for i in threading.enumerate())


class AutowareNode:
    def __init__(self):
        # ZMQ
        context = zmq.Context()
        self.poller = zmq.Poller()
        # Create zmq subscribers
        self.carstate_sock = messaging.sub_sock(
            context,
            service_list['carState'].port,
            conflate=True,
            poller=self.poller)
        # Create zmq publishers
        self.autoware_socket = messaging.pub_sock(
            context, service_list['autoware'].port)
        self.autoware_cmd_socket = messaging.pub_sock(
            context, service_list['autowareCommand'].port)
        self.geofencing_socket = messaging.pub_sock(
            context, service_list['autowareGeofencing'].port)
        self.obstacle_socket = messaging.pub_sock(
            context, service_list['autowareObstacles'].port)
        self.navsatfix_socket = messaging.pub_sock(
            context, service_list['autowareNavSatFix'].port)
        self.twist_socket = messaging.pub_sock(
            context, service_list['autowareTwist'].port)
        self.quat_socket = messaging.pub_sock(
            context, service_list['autowareQuaternion'].port)
        self.gpsd_track_pub = messaging.pub_sock(
            context, service_list['gpsdTrack'].port)
        self.gpsd_speed_pub = messaging.pub_sock(
            context, service_list['gpsdSpeed'].port)
        self.gpsd_status_pub = messaging.pub_sock(
            context, service_list['gpsdStatus'].port)

        # ROS (will block if no roscore found)
        rospy.init_node('autoware_openpilot_bridge', anonymous=True)
        # Set up ROS subscribers
        rospy.Subscriber(
            "/twist_cmd",
            TwistStamped,
            self.ros_callback_speedyaw)
        rospy.Subscriber(
            "/ctrl_cmd",
            ControlCommandStamped,
            self.ros_callback_control_command)
        rospy.Subscriber(
            "geo_fench_coordinate",
            NavSatFix,
            self.ros_callback_geofencing)
        rospy.Subscriber(
            "/lidar_obstacle_distance",
            Float32,
            self.ros_callback_obstacles)
        rospy.Subscriber(
            # "an_device/NavSatFix",
            "gpsd/NavSatFix",
            NavSatFix,
            self.ros_callback_navsatfix)
        rospy.Subscriber(
            "an_device/Twist",
            Twist,
            self.ros_callback_twist)
        rospy.Subscriber(
            "an_device/Imu",
            Imu,
            self.ros_callback_imu)
        rospy.Subscriber(
            "gpsd/Heading",
            Float32,
            self.ros_callback_gpsd_track)
        rospy.Subscriber(
            "gpsd/Speed",
            Float32,
            self.ros_callback_gpsd_speed)
        rospy.Subscriber(
            "gpsd/Has3dFix",
            Bool,
            self.ros_callback_gpsd_status)
        # Set up ROS Publisher
        self.pub_twist = rospy.Publisher(
            '/prius/twist', TwistStamped, queue_size=10)

        # Initialize running state
        self.running = True

    def __del__(self):
        """ Destructor """
        self.running = False

    def ros_callback_speedyaw(self, data):
        """
        :param data: geometry_msgs.msg.TwistStamped message containing speed and yaw rate
        :return:
        """
        contr_msg = messaging.new_message()
        contr_msg.init('autoware')

        target_speed = data.twist.linear.x
        target_yaw_rate = data.twist.angular.z

        # Send the message
        contr_msg.autoware.targetSpeed = target_speed
        contr_msg.autoware.targetYawRate = target_yaw_rate
        self.autoware_socket.send(contr_msg.to_bytes())

    def ros_callback_control_command(self, data):
        """
        :param data: autoware_msgs.msg.ControlCommandStamped message containing speed, acceleration and steering angle
        :return:
        """
        contr_msg = messaging.new_message()
        contr_msg.init('autowareCommand')

        # Send the message
        contr_msg.autowareCommand.targetSpeed = data.cmd.linear_velocity
        contr_msg.autowareCommand.targetAcceleration = data.cmd.linear_acceleration
        contr_msg.autowareCommand.targetSteering = data.cmd.steering_angle
        self.autoware_cmd_socket.send(contr_msg.to_bytes())

    def ros_callback_geofencing(self, data):
        """
        :param data: NavSatFix message containing position of pedestrian in way
        :return:
        """
        geof_msg = messaging.new_message()
        geof_msg.init('autowareGeofencing')

        # Send the message
        geof_msg.autowareGeofencing.pedestrianInWay = 1
        self.geofencing_socket.send(geof_msg.to_bytes())

    def ros_callback_obstacles(self, data):
        """
        :param data: geometry_msgs.msg.TwistStamped message containing speed and yaw rate
        :return:
        """
        obst_msg = messaging.new_message()
        obst_msg.init('autowareObstacles')

        obstacle_distance = data.data
        if obstacle_distance > 0:
            obstacle_detected = 1
        else:
            obstacle_detected = 0

        # Send the message
        obst_msg.autowareObstacles.obstacleDetection = obstacle_detected
        obst_msg.autowareObstacles.obstacleDistance = obstacle_distance
        self.obstacle_socket.send(obst_msg.to_bytes())

    def ros_callback_navsatfix(self, data):
        """
        :param data: NavSatFix message of vehicle gnss position
        :return:
        """
        navsatfix_msg = messaging.new_message()
        navsatfix_msg.init('autowareNavSatFix')

        # Send the message
        navsatfix_msg.autowareNavSatFix.latitude = data.latitude
        navsatfix_msg.autowareNavSatFix.longitude = data.longitude
        navsatfix_msg.autowareNavSatFix.altitude = data.altitude
        self.navsatfix_socket.send(navsatfix_msg.to_bytes())

    def ros_callback_twist(self, data):
        """
        :param data: TwistStamped message of vehicle gnss position
        :return:
        """
        twist_msg = messaging.new_message()
        twist_msg.init('autowareTwist')

        # Send the message
        twist_msg.autowareTwist.velocity = data.linear.x
        twist_msg.autowareTwist.yawrate = data.angular.z
        self.twist_socket.send(twist_msg.to_bytes())

    def ros_callback_imu(self, data):
        """
        :param data: Imu message of vehicle gnss orientation
        :return:
        """
        quat_msg = messaging.new_message()
        quat_msg.init('autowareQuaternion')

        # Send the message
        quat_msg.autowareQuaternion.x = data.orientation.x
        quat_msg.autowareQuaternion.y = data.orientation.y
        quat_msg.autowareQuaternion.z = data.orientation.z
        quat_msg.autowareQuaternion.w = data.orientation.w
        self.quat_socket.send(quat_msg.to_bytes())

    def ros_callback_gpsd_track(self, data):
        """
        :param data: Bool msg containing gpsd status
        :return:
        """
        gpsd_track_msg = messaging.new_message()
        gpsd_track_msg.init('gpsdTrack')

        # Send the message
        gpsd_track_msg.gpsdTrack.track = data.data
        self.gpsd_track_pub.send(gpsd_track_msg.to_bytes())

    def ros_callback_gpsd_speed(self, data):
        """
        :param data: Bool msg containing gpsd speed
        :return:
        """
        gpsd_speed_msg = messaging.new_message()
        gpsd_speed_msg.init('gpsdSpeed')

        # Send the message
        gpsd_speed_msg.gpsdSpeed.speed = data.data
        self.gpsd_speed_pub.send(gpsd_speed_msg.to_bytes())

    def ros_callback_gpsd_status(self, data):
        """
        :param data: Bool msg containing gpsd status
        :return:
        """
        gpsd_status_msg = messaging.new_message()
        gpsd_status_msg.init('gpsdStatus')

        # Send the message
        gpsd_status_msg.gpsdStatus.hasFix = data.data
        self.gpsd_status_pub.send(gpsd_status_msg.to_bytes())

    def run(self):
        while is_main_thread_active() and self.running:
            for socket, event in self.poller.poll(0):
                if socket is self.carstate_sock:
                    carstate_data = messaging.recv_one(socket)
                    car_state = TwistStamped()
                    car_state.twist.linear.x = carstate_data.carState.vEgo
                    car_state.twist.linear.y = 0.0
                    car_state.twist.linear.z = 0.0
                    car_state.twist.angular.x = 0.0
                    car_state.twist.angular.y = 0.0
                    car_state.twist.angular.z = carstate_data.carState.yawRate
                    self.pub_twist.publish(car_state)
            time.sleep(0.01)


if __name__ == "__main__":
    autoware_node = AutowareNode()
    autoware_node.run()
