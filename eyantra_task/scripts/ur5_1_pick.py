#!/usr/bin/env python
"""
Node for the ur5_1 arm to do the job of picking of packages from the shelf and place them on 
the conveyor belt according to the orders recieved

At first the detection of all the packacges on the shelf is done using the constum module qr and their 
details are pushed to the "Inventory" sheet through Action-Client server , after that the planned and
saved trajectories are loaded using a function Initialise_plan. Now whenever a new order is placed 
on the MQTT topic "/eyrc/vb/isPicuTP/orders", it is sorted according to the their description and 
append to dicitonary with key-value pair priority and order id respectively. Ur5_1 start checking 
for the orders from higher to lower priority and proceed according to it , picking package from 
shelf and placing them on conveyor belt . The details of the dispatched order are than updated in the 
"DispatchedOrder" sheet by sending a goal to the Action-Client server.
"""
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg
import json
import actionlib

import yaml
import os
import sys
import math
import time
import datetime

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_ros_iot_bridge.msg import msgMqttSub

from eyantra_task.msg  import DispatchOrder

from decoder import qr
from actionclient import action_client as ac
from packageinfo import info


class Ur5Moveit(object):
    """
    Initialize necessary objects to operate a UR5 arm

    Initialize trajectory publisher, execute client, planning scene interface,
    move group commander, vacuum gripper service and other necessary things
    for proper interface of Ur5 arm with Gazebo, MoveIt! and RViz

    :param arg_robot_name: Name of Ur5 arm to control
    :type arg_robot_name: str

    :Example:

    ur5_1 = Ur5Moveit('ur5_1')
    """

    # pylint: disable=too-many-instance-attributes

    # Constructor
    def __init__(self, arg_robot_name):

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns +
                                                      "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns +
                                                          "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns +
                                                             '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        self._execute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns +
                                                                       '/execute_trajectory',
                                                                       moveit_msgs.msg.
                                                                       ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self.vacuum_gripper_service = '/eyrc/vb/ur5/activate_vacuum_gripper/' + arg_robot_name
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Set joint angles of arm to pre defined joint angles

        :param arg_list_joint_angles: list of goal joint angles of UR5 arm in radians
        :type arg_list_joint_angles: list
        :return: Success flag
        :rtype: bool
        """
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.execute(self._computed_plan, wait=True)

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        Try to set the joint angles again and again until success or max attempts exhausted

        :param arg_list_joint_angles: list of goal joint angles of UR5 arm in radians
        :type arg_list_joint_angles: list
        :param arg_max_attempts: maximum number of attempts to try
        :type arg_max_attempts: int
        :return: None
        """
        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))

    def attach_box_in_gazebo(self):
        """
        Activate vacuum gripper of arm in gazebo

        :return: None
        """
        rospy.wait_for_service(self.vacuum_gripper_service)
        try:
            activate_vacuum_gripper = rospy.ServiceProxy(self.vacuum_gripper_service, vacuumGripper)
            resp1 = activate_vacuum_gripper(True)
            return resp1.result
        except rospy.ServiceException as exception:
            print("Service call failed: %s" % exception)

    def detach_box_in_gazebo(self):
        """
        Deactivate vacuum gripper of arm in gazebo

        :return: None
        """
        rospy.wait_for_service(self.vacuum_gripper_service)
        try:
            activate_vacuum_gripper = rospy.ServiceProxy(self.vacuum_gripper_service, vacuumGripper)
            resp1 = activate_vacuum_gripper(False)
            return resp1.result
        except rospy.ServiceException as exception:
            print("Service call failed: %s" % exception)

    def play_saved_trajectory(self, trajectory):
        """
        Play a trajectory which was saved before

        :param trajectory: Trajectory to be played
        :type trajectory: JointTrajectory
        :return: success flag
        :rtype: bool
        """
        ret = self._group.execute(trajectory)
        return ret

    def hard_play_saved_trajectory(self, trajectory, arg_max_attempts):
        """
        Try playing a trajectory again and again until success or max attempts exhausted

        :param trajectory: Trajectory to be played
        :type trajectory: JointTrajectory
        :param arg_max_attempts: Max number of attempts to try again in case of failure
        :type arg_max_attempts: int
        :return: success flag
        :rtype: bool
        """
        number_attempts = 0
        flag_success = False

        while number_attempts <= arg_max_attempts and flag_success is False:
            number_attempts += 1
            flag_success = self.play_saved_trajectory(trajectory)
            rospy.logwarn("attempts: {}".format(number_attempts))

        return True

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class Ur5communication(object):
    """
    Class used to communicate with ur5 arm by publishing dispatched order details on 
    ROS Topic '/eyrc/vb/package/inform'
    """
    #Initialise 
    def __init__(self):
        #Publisher 
        self._publish = rospy.Publisher('/eyrc/vb/package/inform', DispatchOrder, queue_size=1)

    def send_to_ur52(self , arg_id , arg_priority , arg_order , agr_dispatched , arg_time , arg_color):
        """
        Function publish the dispatched order details on ROS Topic
        :param arg_id: Order Id
        :param arg_priority: Order Priority
        :param arg_order: Order details
        :param arg_dispatched: no. of dispatched order
        :param arg_time: Dispatched time
        :param arg_color: order color
        :return: None
        """
        obj_msg = DispatchOrder()
        obj_msg.OrderId = arg_id
        obj_msg.priority = arg_priority
        obj_msg.order = arg_order
        obj_msg.dispatched = agr_dispatched
        obj_msg.time = arg_time
        obj_msg.diccolor = arg_color 

        self._publish.publish(obj_msg)

    def get_rostime(self):
        """ Function to get current ros time"""
        time_now = rospy.get_time()
        return time_now

class IncomingOrder:
    """
    Class to handle the incoming order which is  published on the MQTT topic '/eyrc/vb/isPicuTP/orders'
    """    
    #Intialise
    def __init__(self):

        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        rospy.loginfo(param_config_iot)
        self._order_received = 0
        self._orders_dic = { 'HP':[] , 'MP':[] , 'LP':[] }
        self._orders_list = {}

        self._subscribe = rospy.Subscriber(self._config_mqtt_sub_cb_ros_topic , msgMqttSub , self.order_callback)


    def order_callback(self, order):
        """
        This is a callback function , it is called everytime when a order
        is published on MQTT topic '/eyrc/vb/isPicuTP/orders'
        :param order: Order details published on MQTT
        :return :None
        """
        rospy.loginfo('Order Received')        
        self._order_received = self._order_received + 1
        order_message = order.message
        self._orders = json.loads(order_message)

        agr_order_id = self._orders["order_id"]
        self._orders_list[agr_order_id] = self._orders

        if self._orders["item"] == "Medicine":
            self._orders_dic['HP'].append(agr_order_id)

        elif self._orders["item"] == "Food" :
            self._orders_dic['MP'].append(agr_order_id)

        elif self._orders["item"] == "Clothes" :
            self._orders_dic['LP'].append(agr_order_id)

    def wait_till_first_order(self):
        """This function wait till first order is recieved"""
        print("Checking For 0rders ....")
        while self._order_received == 0 :
            continue

def saved_trajectory(arg_pkg_dic):
    """
    Function return the list of sorted names of the saved trajectories
    file in 'eyantra_task/config/saved_trajectories/'folder according 
    to priority of the packages on shelf
    :param arg_pkg_dic: Dictionary of packages colour
    :type arg_pkg_dic: dict
    :return: List of Saved plans names
    :rtype: list
    """
    saved_plan_home_to_pkg = { 'HP':[] , 'MP':[] , 'LP':[] }
    saved_plan_pkg_to_home = { 'HP':[] , 'MP':[] , 'LP':[] }

    packages_name = sorted(arg_pkg_dic.keys())
    package_dic = arg_pkg_dic

    for name in packages_name:

        file_name1 = 'home_to_' + name + '.yaml'
        file_name2 =  name + '_to_home.yaml'
    
        if package_dic[name] == 'red':
            saved_plan_home_to_pkg['HP'].append(file_name1)
            saved_plan_pkg_to_home['HP'].append(file_name2)

        elif package_dic[name] == 'yellow':
            saved_plan_home_to_pkg['MP'].append(file_name1)
            saved_plan_pkg_to_home['MP'].append(file_name2)

        elif package_dic[name] == 'green':
            saved_plan_home_to_pkg['LP'].append(file_name1)
            saved_plan_pkg_to_home['LP'].append(file_name2)

    saved_plans = [ saved_plan_home_to_pkg , saved_plan_pkg_to_home]

    return saved_plans

def load_trajectory(arg_saved_plans):
    """
    Function return the list of sorted  loaded trajectories and 
    their execution time in gazebo according to the packages on 
    shelf
    :param arg_saved_plans: List of saved trajectories
    :type arg_saved_plans:list
    :return: List of loaded trajectories and execution time
    :rtype: list
    """
    rp = rospkg.RosPack()
    pkg_path = rp.get_path('eyantra_task')
    arg_file_path = pkg_path + '/config/saved_trajectories/'

    loaded_plan_home_to_pkg = { 'HP':[] , 'MP':[] , 'LP':[] }
    loaded_plan_pkg_to_home = { 'HP':[] , 'MP':[] , 'LP':[] }

    exec_time_home_to_pkg = { 'HP':[] , 'MP':[] , 'LP':[] }
    exec_time_pkg_to_home = { 'HP':[] , 'MP':[] , 'LP':[] }

    loaded_plans = [ loaded_plan_home_to_pkg , loaded_plan_pkg_to_home]
    exec_time =[exec_time_home_to_pkg , exec_time_pkg_to_home]

    for i in range(len(arg_saved_plans)):
        saved_plans_keys = list(arg_saved_plans[i].keys())
        for keys in saved_plans_keys :
            saved_plans_name = arg_saved_plans[i][keys]
            for name in saved_plans_name:           
                file_path = arg_file_path + name
                with open(file_path, 'r') as file_open:
                    loaded_plan = yaml.load(file_open)
                    loaded_plans[i][keys].append(loaded_plan)
                    exec_time[i][keys].append(execution_time(loaded_plan))

    plans_loaded = [ loaded_plans , exec_time]

    return plans_loaded


def sort_plans(arg_plans):
    """
    Function return the list of sorted plans according to their 
    execution time 
    :param arg_plans: List of loaded plans and execution time
    :type arg_plans: list
    :return: Sorted plans
    :rtype: list
    """

    sorted_plan_home_to_pkg = { 'HP':[] , 'MP':[] , 'LP':[] }
    sorted_plan_pkg_to_home = { 'HP':[] , 'MP':[] , 'LP':[] }

    sorted_plans = [sorted_plan_home_to_pkg , sorted_plan_pkg_to_home]

    for i in range(len(arg_plans[0])):
        loaded_plans_key = list(arg_plans[0][i].keys())
        for keys in loaded_plans_key:
            list_plans = arg_plans[0][i][keys]
            list_exec =  arg_plans[1][i][keys]
            sorted_exec = sorted(list_exec)
            for j in range(len(sorted_exec)):
                n = list_exec.index(sorted_exec[j])
                sorted_plans[i][keys].append(list_plans[n])

    return sorted_plans

def update_Inventory(arg_packages):
    """
    This function update the Dispatched Order spreadsheet
    :param arg_packages: packages color dictionary
    :type arg_new_order: dictionary
    """
    team_id = "VB#0574"
    unique_id = "isPicuTP"
    action_client = ac.RosIotBridgeActionClient()
    for row in range(0,4):
        for column in range(0,3):
            pkg_name = "pkgn" + str(row) + str(column)
            pkg_color = arg_packages[pkg_name]
            quantity = 1
            parameters = {"id": "Inventory",
                          "Team Id": team_id,
                          "Unique Id": unique_id,
                          "SKU": info.Stock_Keeping_Unit(row,column,pkg_color),
                          "Item": info.package_item(pkg_color),
                          "Priority": info.package_priority(pkg_color),
                          "Storage Number": info.storage_number(row,column),
                          "Cost": info.package_cost(pkg_color),
                          "Quantity": quantity}
            #Send data to google sheet
            action_client.send_goal("http", "POST", "NA", str(parameters))
            action_client.wait_for_goal_to_complete()

def update_the_dispatched_sheet(arg_processing_order, arg_no_of_dispatched_order):
    """
    This function update the Dispatched Order spreadsheet
    :param arg_processing_order: details of current processing order
    :type arg_new_order: dictionary
    :param arg_no_of_dispatched_order: Number of dispatched order
    :type action_client: int
    :return: Dispatched order Details
    :rtype: Dict
    """
    team_id = "VB#0574"
    unique_id = "isPicuTP"
    action_client = ac.RosIotBridgeActionClient()
    current_processing_order = arg_processing_order
    parameters_dispatched = {"id":"OrdersDispatched", 
                             "Team Id": "VB_0574", 
                             "Unique Id": "isPicuTP", 
                             "Order ID": current_processing_order['order_id'] ,  
                             "City": current_processing_order['city']  , 
                             "Item": current_processing_order['item'] , 
                             "Priority": info.order_priority(current_processing_order['item']) , 
                             "Dispatch Quantity": current_processing_order['qty'] , 
                             "Cost": info.order_cost(current_processing_order['item']) ,  
                             "Dispatch Status":"YES" , 
                             "Dispatch Date and Time": get_time_str()}

    #Send data to google sheet
    action_client.send_goal("http", "POST", "NA", str(parameters_dispatched))

    return parameters_dispatched

def get_time_str():
    """Function to get current timestamp"""
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')

    return str_time

def execution_time(moveit_plan):
    """Function to get the execution time of trajectory in gazebo"""
    no_of_joint_points = len(moveit_plan.joint_trajectory.points)
    planned_time = moveit_plan.joint_trajectory.points[no_of_joint_points- 1].time_from_start
    execution_time = planned_time.secs + (planned_time.nsecs)/1000000000.0 

    return execution_time

def main():
    """
    Main function for processing the incoming order
    """

    rospy.init_node('ur5_1_pick', anonymous=True)

    #Wait for package to swan
    rospy.sleep(5)
    #Start communication
    Com = Ur5communication()
    #Detect package on shelf
    package_color_dic = qr.detect_packages()
    #Update Inventory sheet
    update_Inventory(package_color_dic)

    #Planning
    saved_plans = saved_trajectory(package_color_dic)
    load_plans = load_trajectory(saved_plans)
    loaded_list = sort_plans(load_plans)
    
    #Ur5 arm
    ur5_1 = Ur5Moveit('ur5_1')

    #Incoming Order
    Icorder = IncomingOrder()

    # Home joint angles (position over the conveyor belt)
    ur5_1_home_joint_angles = [math.radians(4),
                               math.radians(-109),
                               math.radians(-82),
                               math.radians(-79),
                               math.radians(90),
                               math.radians(-90)]

    #Go to home position
    ur5_1.set_joint_angles(ur5_1_home_joint_angles)

    #No. of dispatched order
    order_dispatched = 0 
   
    #Wait till first order
    Icorder.wait_till_first_order()
    

    while not rospy.is_shutdown():

        #IF dispatched order is less than order received
        if order_dispatched < Icorder._order_received :
         
            #If high priority orders are available
            if len(Icorder._orders_dic['HP']) > 0:
               
                ur5_1.hard_play_saved_trajectory(loaded_list[0]['HP'][0] , 3 )
                del loaded_list[0]['HP'][0]
                ur5_1.attach_box_in_gazebo()
                  
                ur5_1.hard_play_saved_trajectory(loaded_list[1]['HP'][0], 3)
                del loaded_list[1]['HP'][0]
                ur5_1.detach_box_in_gazebo()

                Current_order = Icorder._orders_list[Icorder._orders_dic['HP'][0]]
                          
                order_dispatched = order_dispatched + 1

                dispatched_paramter = update_the_dispatched_sheet( Current_order , order_dispatched)
           
                Com.send_to_ur52( str(Current_order['order_id']) , 'HP' , str(dispatched_paramter) , order_dispatched , Com.get_rostime() , str(package_color_dic))

                del Icorder._orders_list[Icorder._orders_dic['HP'][0]]
                del Icorder._orders_dic['HP'][0]

                
            #If Medium priority orders are availabe
            elif len(Icorder._orders_dic['MP']) > 0:
               
                ur5_1.hard_play_saved_trajectory(loaded_list[0]['MP'][0],3)
                del loaded_list[0]['MP'][0]
                ur5_1.attach_box_in_gazebo()
                  
                ur5_1.hard_play_saved_trajectory(loaded_list[1]['MP'][0],3)
                del loaded_list[1]['MP'][0]
                ur5_1.detach_box_in_gazebo()

                Current_order = Icorder._orders_list[Icorder._orders_dic['MP'][0]]
                          
                order_dispatched = order_dispatched + 1

                dispatched_paramter = update_the_dispatched_sheet( Current_order , order_dispatched)

                Com.send_to_ur52( str(Current_order['order_id']) , 'MP' , str(dispatched_paramter) , order_dispatched , Com.get_rostime() , str(package_color_dic))

                del Icorder._orders_list[Icorder._orders_dic['MP'][0]]
                del Icorder._orders_dic['MP'][0]


            #If Lower Priority are availabe
            elif len(Icorder._orders_dic['LP']) > 0:
               
                ur5_1.hard_play_saved_trajectory(loaded_list[0]['LP'][0],3)
                del loaded_list[0]['LP'][0]
                ur5_1.attach_box_in_gazebo()
                  
                ur5_1.hard_play_saved_trajectory(loaded_list[1]['LP'][0],3)
                del loaded_list[1]['LP'][0]
                ur5_1.detach_box_in_gazebo()

                Current_order = Icorder._orders_list[Icorder._orders_dic['LP'][0]]
                          
                order_dispatched = order_dispatched + 1

                dispatched_paramter= update_the_dispatched_sheet( Current_order , order_dispatched )

                Com.send_to_ur52( str(Current_order['order_id']) , 'LP' , str(dispatched_paramter) , order_dispatched , Com.get_rostime() , str(package_color_dic))

                del Icorder._orders_list[Icorder._orders_dic['LP'][0]]
                del Icorder._orders_dic['LP'][0]
                     
        else:
            #When all order are dispatched
            if order_dispatched == 9:
                rospy.sleep(1)
                break
            else:
                continue
    del ur5_1 
    del Icorder
                 
if __name__ == "__main__":
    main()


