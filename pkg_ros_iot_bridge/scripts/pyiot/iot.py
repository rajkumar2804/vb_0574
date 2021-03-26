"""
This module does the actual job of subscribing and publishing to MQTT topics,
use GET and POST method to transfer data using HTTP protocol on the request of
Action Server.
"""

import time
import json
import requests
import paho.mqtt.client as mqtt  # import the client1


def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url,
                                arg_broker_port,
                                arg_mqtt_topic,
                                arg_mqtt_qos):

    """
    This function is called in a new thread to subscribe to a MQTT topic.
    :param arg_callback_func: Callback function
    :param arg_broker_url: Broker URL
    :param arg_broker_port: Broker Port
    :param arg_mqtt_topic: MQTT topic to subscribe
    :param arg_mqtt_qos: Quality of Service
    :return: Success bool
    """
    # pylint: disable=bare-except
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1)  # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    """
    This function is called in new thread to publish data to a MQTT topic.
    :param arg_broker_url: Broker URL
    :param arg_broker_port: Broker Port
    :param arg_mqtt_topic: MQTT topic name
    :param arg_mqtt_message: Message to publish
    :param arg_mqtt_qos: Quality of Service
    :return: Success bool
    """
    # pylint: disable=bare-except
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1)  # wait

        mqtt_client.loop_stop()  # stop the loop
        return 0
    except:
        return -1


# -----------------  HTTP POST -------------------
def http_post(spread_sheet_id, parameters):
    """
    This function is called to push data to google spreadsheet using HTTP protocol.
    :param spread_sheet_id: Google Spreadsheet ID
    :param parameters: Parameters to push
    :return: Success flag
    """
    # pylint: disable=bare-except
    try:
        response = requests.get("https://script.google.com/macros/s/" +
                                spread_sheet_id +
                                "/exec", params=parameters)
        return response.content
    except:
        return -1
