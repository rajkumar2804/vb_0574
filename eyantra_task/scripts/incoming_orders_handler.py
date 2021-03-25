#!/usr/bin/env python
"""
This node is used to  handle the job of processing the new orders placed

Node to handle any new order placed on MQTT topic. This node process the order
and add details like priority, cost, team ID and unique ID to the JSON object. 
This node subscribes to ROS topic '/ros_iot_bridge/mqtt/sub' where anything published
on MQTT topic is published to get the details of the new order placed. After this a callback
new_order_callback() do the processing of the order and update the Incoming Orders spreadsheet 
via a ROS-IOT bridge by the mean of Action-Client Server.
"""
import rospy
import json

from pkg_ros_iot_bridge.msg import msgMqttSub
from actionclient import action_client as ac
from packageinfo import info

class IncomingOrders(object):
    """
    Class to handle the new incoming orders published on the ROS
    Topic '/ros_iot_bridge/mqtt/sub' after publish on MQTT topic 
    which then  updated to Incoming Order sheet via Ros Action-Client
    service
    """
    #constructor
    def __init__(self):

        #Initialise the Action Cleint
        self._action_client = ac.RosIotBridgeActionClient()
         
        #Load the Configuration Parameters
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

        #Subscribing the Ros topic
        self._subscribe = rospy.Subscriber(self._config_mqtt_sub_cb_ros_topic , msgMqttSub , self.new_order_callback)

    def new_order_callback(self, msgs):
        """
        This is a callback function , it is called everytime a new 
        order is placed on the ROS topic '/ros_iot_bridge/mqtt/sub'
        and process the order further for Updating in the Incoming
        order sheet
        :param msgs: New order details
        :type msgs: msgMqttSub
        :return: None
        """
        rospy.loginfo('Order Received')

        #New order
        new_order = json.loads(msgs.message)

        #update the spreasheet
        update_incoming_orders_sheet( new_order , self._action_client)

        #Wait for goal to complete
        self._action_client.wait_for_goal_to_complete()
        
   
def update_incoming_orders_sheet( arg_new_order , action_client):
    """
    This function update the Incoming Order spreadsheet
    :param arg_new_order: details of new order
    :type arg_new_order: dictionary
    :param action_client: Action client 
    :type action_client: object
    :return: None
    """
    team_id = "VB#0574"
    unique_id = "isPicuTP"

    parameters_order = {"id":"IncomingOrders", 
                               "Team Id": team_id, 
                               "Unique Id": unique_id, 
                               "Order ID":arg_new_order["order_id"], 
                               "Order Date and Time":arg_new_order["order_time"] , 
                               "Item":arg_new_order["item"],
                               "Priority":info.order_priority(arg_new_order["item"]), 
                               "Order Quantity":arg_new_order["qty"] , 
                               "City":arg_new_order["city"], 
                               "Longitude":arg_new_order["lon"] , 
                               "Latitude":arg_new_order["lat"], 
                               "Cost":info.order_cost(arg_new_order["item"])
                          }

    #Send data to spreadsheet
    action_client.send_goal("http", "POST", "NA", json.dumps(parameters_order))

def main():
    """
    Main function to initialize nodes and the IncomingOrders Class
    """
    # Initialize the node
    rospy.init_node("incoming_orders_handler")

    # Wait for all the models to spawn in Gazebo
    while rospy.get_time() < 5:
        pass

    #Initialise the IncomingOrders Object
    IncomingOrders()

    #Loop untill rospy is shutdown
    while not rospy.is_shutdown():
        pass

if __name__ == "__main__":
    main()

