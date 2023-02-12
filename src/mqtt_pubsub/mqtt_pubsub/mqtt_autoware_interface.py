# This code is developed by Alireza & Mahmoud

import threading
import os.path
import enum
import re
import math
import time

from geographiclib.geodesic import Geodesic
import geographiclib.constants
import geographiclib

import rclpy
from rclpy.node import Node

# Importing the required ros2 message types
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion

# Importing the required mqtt python package
import paho.mqtt.client as paho
from paho import mqtt

from rclpy.action import ActionClient
from recordreplay_planner_actions.action import ReplayTrajectory

from ament_index_python import get_package_share_directory

from os import listdir
from os.path import isfile, join


# An enum class representing the software stack being used
class SoftwareStack(enum.Enum):
    Autoware = 1
    AIIM = 2

# Defining the mqtt autoware interface node
class mqttAutowareInterface(Node):
    """
        An interface node that connects the mqtt server to other ros2 nodes. It includes:
        MQTT subscriber: Subscribes to the mqtt server to receive messages sent by the user through phone.
        MQTT publisher: Publishes over the mqtt server to send messages to the user's phone.
        ROS2 publisher: After receiving messages by the mqtt subscriber, they are published to the appropriate ros2 topics.
        ROS2 subscriber: Subscribes to ROS2 topics to receive messages so that it can then be published to the mqtt server.
    """

    def __init__(self):
        super().__init__('mqtt_autoware_interface')
        
        
        #Declaring all parameters 
        self.declare_parameter("originLat",51.44823418857984)
        self.declare_parameter("originLong",5.4824253844144195)
        self.declare_parameter("MQTTPubTime", 2.0)
        self.declare_parameter("username", "umo_mqtt")
        self.declare_parameter("password", "car@mqtt")
        self.declare_parameter("serverURI", "f799eccb4f10439197588da60e6ac2e4.s1.eu.hivemq.cloud")
        self.declare_parameter("serverPort", 8883)
        self.declare_parameter("goalTopic", "avp/goal")
        self.declare_parameter("stackChoiceTopic", "avp/stack")
        self.declare_parameter("missionFeedbackTopic", "avp/feedback")
        self.declare_parameter("vehiclePoseFeedbackTopic", "avp/pose")
        
        # Coordinates of the origin of the map
        self.originLat = self.get_parameter("originLat").get_parameter_value().double_value
        self.originLong = self.get_parameter("originLong").get_parameter_value().double_value

        # Minimum time between 2 MQTT (vehicle pose) messages publication
        self.MQTTPubTime = self.get_parameter("MQTTPubTime").get_parameter_value().double_value
        
        # MQTT broker info
        username = self.get_parameter("username").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        serverURI = self.get_parameter("serverURI").get_parameter_value().string_value
        serverPort = self.get_parameter("serverPort").get_parameter_value().integer_value
        self.goalTopic = self.get_parameter("goalTopic").get_parameter_value().string_value
        self.stackChoiceTopic = self.get_parameter("stackChoiceTopic").get_parameter_value().string_value
        self.missionFeedbackTopic = self.get_parameter("missionFeedbackTopic").get_parameter_value().string_value
        self.vehiclePoseFeedbackTopic = self.get_parameter("vehiclePoseFeedbackTopic").get_parameter_value().string_value
        

        # Which software stack is being used: Autoware or AIIM
        self.softwareStack = SoftwareStack.AIIM

        # Ros2 Publishers, Subscribers & Action servers/clients

        self.publisher_ = self.create_publisher(PoseStamped, '/planning/goal_pose', 10)
        self.trjaectoryFilePathPub = self.create_publisher(String, '/trajectory_file_path', 10)
        self.autoware_subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization/ndt_pose',
                                                              self.autoware_onPoseUpdated, 10)
        self.subscription = self.create_subscription(PoseStamped, '/ndt_pose', self.AIIM_onPoseUpdated, 10)
        self.subscription
        self.autoware_subscription  # To avoid unused warning
        # Creating Action client for replay trajectory action server
        self._action_client = ActionClient(self, ReplayTrajectory, 'replaytrajectory')

        # MQTT Connection Setup
        self.prevPubTime = time.perf_counter()
        self.client = paho.Client(client_id="", userdata=None, protocol=paho.MQTTv5)
        self.client.on_connect = self.MQTT_onConnect

        # enable TLS for secure connection
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)

        # set username and password
        self.client.username_pw_set(username, password)

        # connect to HiveMQ Cloud (default port for MQTT 8883)
        self.client.connect(serverURI, serverPort)
        self.client.on_subscribe = self.MQTT_onSubscribe

        # subscribe to mqtt topics for receiving messages from the phone
        self.client.subscribe(self.goalTopic, qos=2)
        self.client.subscribe(self.stackChoiceTopic, qos=2)
        self.client.loop_start()
        self.client.on_message = self.MQTT_onMessage

        # Setting flags
        self.trajectoryProcessing = False
        self.actionServiceFree = True

        # Creating and populating the lists of all available paths / goal poses
        mqtt_pubsub_pkg_data = get_package_share_directory('mqtt_pubsub') + '/data'

        convert = lambda text: int(text) if text.isdigit() else text
        alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
        onlyfiles = [f for f in listdir(mqtt_pubsub_pkg_data) if isfile(join(mqtt_pubsub_pkg_data, f))]
        onlyfiles.sort(key=alphanum_key)

        self.trajectoryList = []

        for f in onlyfiles:
            self.trajectoryList.append(join(mqtt_pubsub_pkg_data, f))

        self.goalPoseList = []
        self.goalPoseList.insert(1, self.goalPoseCreator(-73.55915832519531, 63.752655029296875, 0.8978381234181558,
                                                         -0.44032567962471186))
        self.goalPoseList.insert(2, self.goalPoseCreator(695.8806762695312, 14.976155281066895, 0.020395455808143385,
                                                         0.9999308184067669))

        
    # region Custom functions
    def MQTT_MessageProccessing(self, msg):
        """
            A method that gets called on a separate concurrently running thread with every mqtt message received,
            it handles all operations related to user inputs

            :param msg: the mqtt message received from the server on any of the 2 subscribed topics
        """

        msg.payload = msg.payload[2:-1]
        # print("Received message: " + msg.payload + " from topic: " + msg.topic)
        minMsg = msg.payload.strip().lower()
        print("Manipulated message: " + minMsg)

        # Populating the original feedback message to be published on the missionFeedbackTopic
        ros2Msg = str("Command: " + msg.payload + " at topic: " + msg.topic + " is invalid")
        # Checking if the message received is on the goalTopic
        if msg.topic == self.goalTopic:

            # Get the parking or plan number to be executed
            self.parkingNum = self.phoneGoalMessageChecker(minMsg)

            # Comparing the parking choice to the available spots. Populating and Publishing the goal pose message to autoware.
            if self.parkingNum > 0:
                # Checking which stack is chosen and ensuring that the parkingNum is within the available lists
                if self.softwareStack == SoftwareStack.Autoware and self.parkingNum <= len(self.goalPoseList):

                    # Populating autoware goal pose message
                    goalPose = self.goalPoseList[self.parkingNum - 1]

                    # Populating mqtt feedback message
                    ros2Msg = str("Car Moving towards Parking " + str(self.parkingNum) + " using autoware stack")

                    # Publishing goal pose to autoware
                    self.publisher_.publish(goalPose)

                elif self.softwareStack == SoftwareStack.AIIM and self.parkingNum <= len(self.trajectoryList):

                    ros2Msg = self.AIIM_pathPicker(self.parkingNum, ros2Msg)

            # In case the user wants to forcefully cancel an ongoing (AIIM) trajectory already loaded and not finished yet
            elif minMsg == "cancel" and self.softwareStack == SoftwareStack.AIIM:
                # if hasattr(self, '_goal_handle'):
                if self.trajectoryProcessing == True:
                    self.get_logger().info('Canceling previous goal')
                    # Cancel the ongoing goal (which is not finished yet)
                    futureCancelGoal = self._goal_handle.cancel_goal_async()
                    futureCancelGoal.add_done_callback(self.replayTrajectory_onGoalForceCancel)
                    # Setting the message to empty string as it will be sent from the callbacks
                    ros2Msg = ""
                else:
                    ros2Msg = "Either no existing goal going on to cancel, or current trajectory is finished (<=threshold)."

        # Checking if the received message is from the stackChoiceTopic and changing the stack accordingly
        elif msg.topic == self.stackChoiceTopic:
            # Changing the software stack variable based on the received message
            if minMsg == "autoware":
                self.softwareStack = SoftwareStack.Autoware
                ros2Msg = str("Software stack changed to " + self.softwareStack.name)
            elif minMsg == "aiim":
                self.softwareStack = SoftwareStack.AIIM
                ros2Msg = str("Software stack changed to " + self.softwareStack.name)
            else:
                ros2Msg = str("Invalid stack name, current stack is " + self.softwareStack.name)

        if ros2Msg != "":
            # Publishing feedback to mqtt server
            self.client.publish(self.missionFeedbackTopic, ros2Msg, 2)
            # Printing the feedback message on console
            print(ros2Msg)

    def AIIM_pathPicker(self, parkingNum, ros2Msg):
        """
            A method that manages how a goal is sent to the replay trajectory action server
            then populates a feedback message (ros2Msg) expressing the success or failure of the method

            :param parkingNum: an int representing the path chosen by the user
            :param ros2Msg: an initial value for the return string of the method
        """
        # Setting the trajectory file path
        # self.trajectoryPath = os.path.expanduser(self.trajectoryList[parkingNum - 1])
        self.trajectoryPath = self.trajectoryList[parkingNum - 1]
        msg = String()
        msg.data = str(self.trajectoryPath)
        self.trjaectoryFilePathPub.publish(msg)
        print("Path is: " + self.trajectoryPath)
        # Checking if the recordreplay action server is free to run the process
        if self.actionServiceFree == True:
            # Send the goal if the action server is free (no ongoing process)
            serverAvailable = self.replayTrajectory_sendGoal(self.trajectoryPath)
            if serverAvailable:
                # Setting the message to empty string as it will be sent from the callbacks
                ros2Msg = ""
            else:
                ros2Msg = "Timeout: Replay action server was not ready, try again"

        else:
            # Checking if the record/replay action server has an ongoing goal.
            # If the previous trajectory already reached the final point then cancel the goal (The action goal never terminates automatically in the AIIM stack)
            # A new goal is sent upon sucessful cancelation of the previous goal through the cancellation callback (Based on the user input)
            if self.trajectoryProcessing == False:
                self.get_logger().info('Canceling goal')
                # Cancel the goal
                future = self._goal_handle.cancel_goal_async()
                # once the goal is cancelled successfully, another goal will be requested from the onGoalCancelled callback
                future.add_done_callback(self.replayTrajectory_onGoalCanceled)

                # Setting the message to empty string as it will be sent from the callbacks
                ros2Msg = ""

            else:
                ros2Msg = str("Car is already executing a path using AIIM stack")

        return ros2Msg

    def goalPoseCreator(self, x, y, w, z):
        """
            A method that populates a goalPose of the type PoseStamped and returns it, the frame_id is set to map
            all other values needed in the PoseStamped are set to the default value

            :param x: a float representing the x coordinate of the pose
            :param y: a float representing the y coordinate of the pose
            :param w: a float representing the quaterion w rotation of the pose
            :param z: a float representing the quaterion z rotation of the pose
        """
        header = Header()
        header.frame_id = "map"
        goalPose = PoseStamped(header=header, pose=Pose(position=Point(x=x, y=y),
                                                        orientation=Quaternion(w=w, z=z)))
        return goalPose

    def phoneGoalMessageChecker(self, minMsg):
        """
            A method that checks the user message for validity and returns a number representing the chosen
            path / goalpose if the message is valid

            :param minMsg: a string representing the unformatted user message
        """
        words = minMsg.split(" ")
        if len(words) > 2:
            return 0

        if words[0] == "parking" or words[0] == "path" or words[0] == "trajectory":
            try:
                parkingNum = int(words[1])
            except:
                parkingNum = 0
            return parkingNum
        else:
            try:
                parkingNum = int(words[0])
            except:
                parkingNum = 0
            return parkingNum

    def MQTT_vehicleCoordinatesPublisher(self, x, y):
        """
            A method that to publish the coordinates to the phone after converting it from local to global coordinates

            :param x: x position of the car in local coordinates
            :param y: y position of the car in local coordinates
        """

        earth = Geodesic.WGS84
        angle = math.degrees(math.atan2(x, y))
        distance = math.sqrt(x * x + y * y)

        result = earth.Direct(self.originLat, self.originLong, angle, distance)
        # mqttFeedbackMsg = "X: " + str(x) + ", Y: " + str(y)
        mqttFeedbackMsg = "lat: " + str(result["lat2"]) + ", lon: " + str(result["lon2"])
        if (time.perf_counter() - self.prevPubTime) > self.MQTTPubTime:
            self.client.publish(self.vehiclePoseFeedbackTopic, mqttFeedbackMsg, 2)
            self.prevPubTime = time.perf_counter()

    # endregion

    # region Pose updated callbacks
    def autoware_onPoseUpdated(self, msg: PoseWithCovarianceStamped):
        """
            Call back for ros2 subscriber (autoware) to retrieve location information from NDT node
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.MQTT_vehicleCoordinatesPublisher(x, y)

    def AIIM_onPoseUpdated(self, msg: PoseStamped):
        """
            Call back for ros2 subscriber (AIIM) to retrieve location information from NDT node
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        self.MQTT_vehicleCoordinatesPublisher(x, y)

    # endregion

    # region Replay Trajectory Action Client functions and callbacks
    def replayTrajectory_sendGoal(self, trajectoryPath):
        """
            Action goal of the recordreplay trajectory node
        """
        goal_msg = ReplayTrajectory.Goal()
        goal_msg.replay_path = trajectoryPath

        serverAvailable = self._action_client.wait_for_server(1)
        if serverAvailable:
            self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                                                                         feedback_callback=self.replayTrajectory_onFeedback)
            self._send_goal_future.add_done_callback(self.replayTrajectory_onGoalResponse)
            self.get_logger().info('Goal sent')
        return serverAvailable

    def replayTrajectory_onGoalResponse(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.client.publish(self.missionFeedbackTopic, "Goal rejected please try again", 2)
            return
        self.actionServiceFree = False
        self._goal_handle = goal_handle
        self.trajectoryProcessing = True
        self.get_logger().info('Goal accepted :)')
        ros2Msg = "Goal accepted, car moving towards parking " + str(self.parkingNum)
        self.client.publish(self.missionFeedbackTopic, ros2Msg, 2)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.replayTrajectory_onGetResult)

    def replayTrajectory_onGetResult(self, future):
        result = future.result().result
        if self.trajectoryProcessing == False:
            self.actionServiceFree = True
        
        self.get_logger().info('Result: {0}'.format(result))

    def replayTrajectory_onFeedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.remaining_length))
        if feedback.remaining_length <= 10:
            self.trajectoryProcessing = False
        elif feedback.remaining_length > 10:
            self.trajectoryProcessing = True

    def replayTrajectory_onGoalCanceled(self, future):
        """
            A callback for cancelling an ongoing action goal and sends a new goal based on self.trajectoryPath (user's new input)
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal canceled')
            self.replayTrajectory_sendGoal(self.trajectoryPath)
        else:
            self.get_logger().info('Goal failed to cancel')
            self.client.publish(self.missionFeedbackTopic, "Goal cancelling failed please try again", 2)

    def replayTrajectory_onGoalForceCancel(self, future):
        """
            A callback for cancelling an ongoing action goal forcefully without sending a new goal right afterwards.
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal forcefully canceled')
            self.client.publish(self.missionFeedbackTopic, "Goal forcefully canceled. Ready to receive a new goal.", 2)
            # So the action service becomes available again and a new goal can be sent by the user.
            # Otherwise if we send a new goal, the replayTrajectory_onGoalCanceled callback inside the AIIM_pathPicker will try to cancel a non existent action goal.
            self.actionServiceFree = True
            # Setting the trajectoryProcessing into False here will not matter for what regards receiving a new goal because actionServiceFree is already True.
            # However, it will prevent double cancelling if the user sends another cancel message. 
            self.trajectoryProcessing = False
        else:
            self.get_logger().info('Goal failed to cancel forcefully')
            self.client.publish(self.missionFeedbackTopic, "Forced goal cancelling failed please try again", 2)

    # endregion

    # region MQTT callbacks
    def MQTT_onMessage(self, client, userdata, msg):
        """
            Callback for receiving messages over the mqtt server.

            :param client: the client itself
            :param userdata: userdata is set when initiating the client, here it is userdata=None
            :param msg: the message with topic and payload
        """
        # Todo: Where does this b come from originally? Figure it out.
        # Modifying the received message to remove any formatting and simplify the message
        msg.payload = str(msg.payload)
        # print("Received Raw message: " + msg.payload + " from topic: " + msg.topic)

        t = threading.Thread(target=self.MQTT_MessageProccessing, args=(msg,))
        t.start()

    def MQTT_onSubscribe(self, client, userdata, mid, granted_qos, properties=None):
        """
            Prints a reassurance for successfully subscribing

            :param client: the client itself
            :param userdata: userdata is set when initiating the client, here it is userdata=None
            :param mid: variable returned from the corresponding publish() call, to allow outgoing messages to be tracked
            :param granted_qos: this is the qos that you declare when subscribing, use the same one for publishing
            :param properties: can be used in MQTTv5, but is optional
        """
        print("Subscribed: " + str(mid) + " " + str(granted_qos))

    def MQTT_onConnect(self, client, userdata, flags, rc, properties=None):
        """
            Prints the result of the connection with a reasoncode to stdout ( used as callback for connect )
            
            :param client: the client itself
            :param userdata: userdata is set when initiating the client, here it is userdata=None
            :param flags: these are response flags sent by the broker
            :param rc: stands for reasonCode, which is a code for the connection result
            :param properties: can be used in MQTTv5, but is optional
        """
        print("CONNACK received with code %s." % rc)
    # endregion


def main(args=None):
    rclpy.init(args=args)

    mqttInterface = mqttAutowareInterface()

    rclpy.spin(mqttInterface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mqttInterface.client.loop_stop()
    mqttInterface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
