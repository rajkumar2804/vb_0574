#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

"""
This node is responsible for running an Action Server which serves as a bridge between
Ros and IoT.

Any task like getting data from a MQTT topic to pushing data on spreadsheets is
achieved by this Action Server. All the necessary parameters are first loaded from the
parameter server. Then whenever a client sends a goal to this Action Server, a new thread
is started to process the goal asynchronously. New thread is an essential requirement to process
multiple goals at once.
"""

from __future__ import print_function

import threading
import rospy
import actionlib

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pkg_ros_iot_bridge.msg import msgMqttSub    # Message Class for MQTT Subscription Messages

from pyiot import iot                            # Custom Python Module to perform MQTT Tasks


class RosIotBridgeActionServer(object):
    """
    Initialize the Action Server and load all necessary parameters from the parameter server.

    This class can subscribe and publish to any MQTT topic. If a MQTT topic is subscribed,
    then any data published in that topic is published to another ROS topic
    "/ros_iot_bridge/mqtt/sub" so that other ROS nodes can access that data for their use. It can
    also get and post using HTTP protocol to update google spreadsheets.
    """

    # pylint: disable=too-many-instance-attributes

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
        * self.on_goal - It is the function pointer which points to a function which will be
                         called when the Action Server receives a Goal.

        * self.on_cancel - It is the function pointer which points to a function which will be
                           called when the Action Server receives a Cancel Request.
        '''

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']
        self._config_sheet_id_eyantra = param_config_iot['google_apps']['spread_sheet_id_eyantra']
        print(param_config_iot)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS
        # Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get
        # messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic,
                                               msgMqttSub,
                                               queue_size=10)

        # Subscribe to MQTT Topic (eyrc/isPicuTP/iot_to_ros) which is defined in config_pyiot.yaml.
        # self.mqtt_sub_callback() function will be called when there is a message from
        # MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, _client, _userdata, message):
        """
        This function is called whenever something is published to the subscribed MQTT topic
        and publish the same data to a ROS topic so that other ROS nodes can access it

        :param _client: Client
        :param _userdata: User Data
        :param message: Message published on the MQTT topic
        :type message: msgMqttSub
        :return: None
        """
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        """
        This function is called whenever a new goal is received by the Action Server

        :param goal_handle: Unique ID of the goal
        :return: None
        """
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt" or goal.protocol == "http":

            if goal.mode == "pub" or goal.mode == "sub" or goal.mode == "POST":
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client (Asynchronous Processing)
                # 'self.process_goal' - is the function pointer which points to a function that
                #                       will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        """
        This function is called to create a separate thread to process the Goal.

        :param goal_handle: Unique ID of the goal
        :return: None
        """

        # pylint: disable=too-many-branches

        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        # To handle HTTP goals
        elif goal.protocol == "http":
            rospy.logwarn("HTTP")

            if goal.mode == "POST":
                # In case of HTTP request, goal.topic represents the spreadsheet
                if goal.topic == 'eyantra':
                    ret = iot.http_post(self._config_sheet_id_eyantra,
                                        goal.message)
                else:
                    ret = iot.http_post(self._config_spread_sheet_id,
                                        goal.message)

                if ret == 'success':
                    rospy.loginfo("Spreadsheet updated successfully")
                    result.flag_success = True
                else:
                    rospy.loginfo("Error in updating spreadsheet")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success is True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        """
        This function will be called when Goal Cancel request is send to the Action Server.

        :param goal_handle: Unique ID of the goal
        :return: None
        """
        # pylint: disable=no-self-use
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Goal with goal id ", goal_id, " has been cancelled")


# Main
def main():
    """
    Main function to create an object of class RosIotBridgeActionServer to initialize the Action Server.

    :return: None
    """
    rospy.init_node('node_action_server_ros_iot_bridge')

    _action_server = RosIotBridgeActionServer()

    rospy.spin()


if __name__ == '__main__':
    main()
