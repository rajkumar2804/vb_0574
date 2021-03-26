#!/usr/bin/env python

"""
This module handle the processing of the different goals from the clients which
have protocol,mode,topic and message, send them to the  Action Server which act 
as bridge between the ROS and IOT. The Action Server will then send feedback which 
have percentage_complete  while it is processing the goal and result which have 
flag_success after processing the Goal.
"""
import rospy
import actionlib
import time
import requests

from pkg_ros_iot_bridge.msg import msgRosIotAction     
from pkg_ros_iot_bridge.msg import msgRosIotGoal        
from pkg_ros_iot_bridge.msg import msgRosIotResult 

  
class RosIotBridgeActionClient(object):
    """
    Action Client which proceed the goals from the client to 
    Action Server and act as a bridge between ROS and IOT to 
    update the google spreasheets
    """

    #Constructor 
    def __init__(self):

       # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}
        
        #goal status varaible
        self._goal_status = 0

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
        rospy.loginfo("Action server up, we can send goals.")
    
    def on_transition(self, goal_handle):
        """
        This function will be called when there is change of state in the Action Client State
        Machine

        :param goal_handle: unique id of goal
        :return: None
        """       
        #from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            pass
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:           
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if (result.flag_success == True):
                self._goal_status = 7
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                pass

    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        """
        This function is used to send Goals to Action Server

        :param arg_protocol: IoT protocol to follow (HTTP, MQTT, etc.)
        :type arg_protocol: str
        :param arg_mode: Mode whether to publish or subscribe
        :type arg_mode: str
        :param arg_topic: Topic to subscribe or publish to
        :type arg_topic: str
        :param arg_message: Message which is push or publish 
        :type arg_message: str
        :return: goal handle
        """   
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)

        self._goal_handles['Goal'] = goal_handle

        return goal_handle

    def wait_for_goal_to_complete(self):
        """This function will wait untill the given goal is successfully completed."""
        #Wait for the goal to complete
        while(self._goal_status != 7):
            pass
        #Reset the goal status
        self._goal_status = 0
