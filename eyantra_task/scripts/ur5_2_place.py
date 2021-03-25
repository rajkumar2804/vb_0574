#! /usr/bin/env python

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg

import yaml
import os
import sys
import math
import time
import ast
import datetime

from std_srvs.srv import Empty

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from eyantra_task.msg  import DispatchOrder

from hrwros_gazebo.msg import LogicalCameraImage

from actionclient import action_client as ac
from packageinfo import info

class DispatchedOrder:
  
    #constructor 
    def __init__(self):
        # Initialize ROS Topic Publication
        # Incoming message from Logical camera 2 will be published on a ROS Topic (/eyrc/vb/logical_camera_2).
        #Rate update =10
        self._topic = "/eyrc/vb/logical_camera_2"
        self._sub_topic = "/eyrc/vb/package/inform"
        self._message = LogicalCameraImage
        # Initialize flag 
        self.model_detect = 0
        self._package_detect = 0
        self._package_found = 0
        self.conveyor_belt_on = On_conveyor_belt()
        #variables
        self._no_of_dispatched = 0
        self._dispatched_order = {}
        self._dispatched_list = []
        self._color_dic = {}
        self._dispatched_time = {}
        self._shipped_time = {}
        self._time_elapsed = {}
        self._going_position = ' '

    def get_dispatched(self):
        #dispatched subscriber
        self._sub = rospy.Subscriber(self._sub_topic , DispatchOrder , self.dispatched_callback)


    def dispatched_callback(self,data):
        #order_recieved
        rospy.loginfo( 'Order' + str(data.dispatched) + ' Dispatched')
        self._order_id = data.OrderId
        self._no_of_dispatched = data.dispatched
        self.priority = data.priority
        self._dispatched_order[self._order_id] = ast.literal_eval(data.order)
        self._dispatched_list.append(self._order_id)
        if self._no_of_dispatched == 1:
            self._color_dic = ast.literal_eval(data.diccolor)
        self._dispatched_time[self._order_id] = data.time



    #Logical subscribtion
    def logical_camera(self):
        #Subscribe to logical camera topic and attach a callback function self.logical_callback to it, when a message is published on topic it will call the
        #function self.logical_callback
        self.sub = rospy.Subscriber(self._topic,self._message , self.logical_callback)
        
    def logical_callback(self , data):
        #Extract data from the Topic in the form of list 
        self.Model = data.models

        #If the list is true , create the list of package name and their respective position
        if self.Model :
            self.model_detect = 1

            for i in range(0, len(self.Model)):
                Model_name = self.Model[i].type
                Model_pose_y= self.Model[i].pose.position.y

                name_key = 'pkgn'+ Model_name[-2]+ Model_name[-1]

                if len(self._dispatched_list)>0:
                    if len(self._dispatched_time)>0 and len(self._shipped_time) > 0:    
                        self._time_elapsed[self._dispatched_list[0]] = self._dispatched_time[self._dispatched_list[0]] + 6.7-self._shipped_time[self._dispatched_list[0]]
                        stoping_range = prefered_stoping_range(self._going_position , self._time_elapsed[self._dispatched_list[0]] ,self._dispatched_order[self._dispatched_list[0]]["Priority"])
                        if name_key in list(self._color_dic.keys()) :
                            if (Model_pose_y < stoping_range[0] and Model_pose_y> stoping_range[1]):
                                self._package_detect = 1
                                break
                            else:
                                self._package_detect = 0
                                continue

            #If package is found and conveyor belt is on turn it off and set flag  self.conveyor_belt_on = 0
            if self._package_detect ==1 and self.conveyor_belt_on == 1 :
                self.conveyor_belt_on = Off_conveyor_belt() 
 
           #If package is not  found and conveyor belt is off turn it on and set flag  self.conveyor_belt_on = 1
           # Start conveyor belt at 100% speed
            elif self.conveyor_belt_on == 0 and self._package_detect== 0 :      
                self.conveyor_belt_on = On_conveyor_belt()

            if self._package_detect ==1 and self.conveyor_belt_on == 0 :
                self._package_found = 1
            else:
                self._package_found = 0

                                                      
        #If list is not true hence no package is detected turn the conveyor belt set self.conveyor_belt_on = 1
        # Start conveyor belt at 100% speed
        else:
            self.model_detect = 0
            if self.conveyor_belt_on == 0 :                
                self.conveyor_belt_on = On_conveyor_belt()  
            else:
                pass


    def wait_to_dispatched(self):
        while self._no_of_dispatched < 1:
            pass    
   

    def wait_package_to_arrived(self):
        while self._package_found != 1:
            pass


def On_conveyor_belt():
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
        power= rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        resp1 = power(100)
        response = 1
        return response
    except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def Off_conveyor_belt():
    rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
    try:
        power= rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        resp1 = power(0)
        response = 0
        return response
    except rospy.ServiceException as e:
         print("Service call failed: %s"%e)


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

        ros_pkg = rospkg.RosPack()
        self._pkg_path = ros_pkg.get_path('eyantra_task')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

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


def get_time_str():
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')

    return str_time

def estimated_time_to_delivery(arg_priority):

    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    if arg_priority == 'HP':
        new_value = value + datetime.timedelta(days=1)
    elif arg_priority =='MP':
        new_value =  value + datetime.timedelta(days=3)
    elif arg_priority =='LP':
        new_value = value + datetime.timedelta(days=5)

    str_time = new_value.strftime('%Y-%m-%d')

    return str_time


def updated_the_shipped_sheet(current_processing_order,  no_of_shipped_order):

    action_client = ac.RosIotBridgeActionClient()
    parameters_shipped = {"id":"OrdersShipped", 
                          "Team Id": "VB_0574", 
                          "Unique Id": "isPicuTP", 
                          "Order ID": current_processing_order['Order ID'] ,  
                          "City": current_processing_order['City']  , 
                          "Item": current_processing_order['Item'] , 
                          "Priority": current_processing_order["Priority"]  , 
                          "Shipped Quantity": current_processing_order['Dispatch Quantity'] , 
                          "Cost": info.order_cost(current_processing_order['Item'])  ,  
                          "Shipped Status":"YES" , 
                          "Shipped Date and Time": get_time_str() ,
                          "Estimated Time of Delivery": estimated_time_to_delivery(current_processing_order["Priority"])}

    action_client.send_goal("http", "POST", "NA", str(parameters_shipped))


def prefered_home_position( current_shipped_position , current_order_priority , time_elapsed_from_dispatched ):

    if current_shipped_position == 'zero' :
        if current_order_priority == 'HP' or current_order_priority == 'MP':
            prefered_home_position = 'home1'
        elif current_order_priority == 'LP':
            prefered_home_position = 'home3'
        return prefered_home_position

    elif current_shipped_position == 'red':
        if current_order_priority == 'HP' or current_order_priority == 'MP':
            if time_elapsed_from_dispatched >=1.8 :
                prefered_home_position = 'home1'
            elif time_elapsed_from_dispatched < 1.8:
                prefered_home_position = 'home2'    
        elif current_order_priority == 'LP':
            prefered_home_position = 'home3'    
        return prefered_home_position

    elif current_shipped_position == 'yellow':
        if current_order_priority == 'HP' or current_order_priority == 'MP':
            if time_elapsed_from_dispatched >= 3.5 :
                prefered_home_position = 'home1'
            elif time_elapsed_from_dispatched < 3.5:
                prefered_home_position = 'home2'    
        elif current_order_priority == 'LP':
            prefered_home_position = 'home3'    
        return prefered_home_position

    elif current_shipped_position == 'green':
        if current_order_priority == 'HP' or current_order_priority == 'MP':
            if time_elapsed_from_dispatched >= 3:
                prefered_home_position = 'home1'
            elif time_elapsed_from_dispatched < 3:
                prefered_home_position = 'home2'    
        elif current_order_priority == 'LP':
            prefered_home_position = 'home3'    
        return prefered_home_position


def prefered_stoping_range( current_shipped_position , elapsed_time , dispatched_order_priority):

    home1_range =  (0.55 , 0.45)
    home2_range =  (0.18 , 0.08)
    home3_range =  (-0.20 , -0.30)

    if current_shipped_position == 'zero':
        if dispatched_order_priority == 'HP' or dispatched_order_priority == 'MP':
            return home1_range
        elif dispatched_order_priority == 'LP':
            return home3_range        

    if current_shipped_position == 'red':
        if dispatched_order_priority == 'HP' or dispatched_order_priority == 'MP':
            if elapsed_time >= 1.8:
                return home1_range
            elif elapsed_time <1.8:
                return home2_range
        elif dispatched_order_priority == 'LP':
            return home3_range

    if current_shipped_position == 'yellow':
        if dispatched_order_priority == 'HP' or dispatched_order_priority == 'MP':
            if elapsed_time >= 3.5:
                return home1_range
            elif elapsed_time <3.5:
                return home2_range
        elif dispatched_order_priority == 'LP':
            return home3_range   
    
    if current_shipped_position == 'green':
        if dispatched_order_priority == 'HP' or dispatched_order_priority == 'MP':
            if elapsed_time >= 3:
                return home1_range
            elif elapsed_time <3:
                return home2_range
        elif dispatched_order_priority == 'LP':
            return home3_range 
  

def Initialise_plan():

    rp = rospkg.RosPack()
    pkg_path = rp.get_path('practice_pkg')
    arg_file_path = pkg_path + '/config/ur5_2_trajectories/'

    saved_plans_to_homes = { 'zero':   {'home1': 'zero_to_home1.yaml' ,  
                                        'home3':'zero_to_home3.yaml'},
                             'red' :   {'home1':  'red_to_home1.yaml'  , 
                                        'home2': 'red_to_home2.yaml' , 
                                        'home3' : 'red_to_home3.yaml'} ,
                             'yellow': {'home1':  'yellow_to_home1.yaml'  , 
                                        'home2': 'yellow_to_home2.yaml' , 
                                        'home3' : 'yellow_to_home3.yaml'},
                             'green': { 'home1':  'green_to_home1.yaml'  , 
                                        'home2': 'green_to_home2.yaml' , 
                                        'home3' : 'green_to_home3.yaml'}}

    saved_homes_to_bin =  { 'home1' : { 'red' : 'home1_to_red.yaml' , 
                                       'yellow': 'home1_to_yellow.yaml' } ,
                            'home2' : { 'red' : 'home2_to_red.yaml' , 
                                       'yellow': 'home2_to_yellow.yaml' } ,
                            'home3' : { 'green' : 'home3_to_green.yaml' } }


    loaded_home = { 'zero': { 'home1': ' ' , 'home3': ' '} ,  
                    'red' : { 'home1': ' ' , 'home2': ' ' , 'home3': ' ' } ,
                    'yellow' : { 'home1': ' ' , 'home2': ' ' , 'home3': ' ' } ,
                    'green':  { 'home1': ' ' , 'home2': ' ' , 'home3': ' ' } }

    loaded_pkg = { 'home1': {'red' : ' ' , 'green': ' ' } , 'home2':{'red':' ' , 'green': ' '} , 'home3': { 'green':' '} }

    saved_plans  = [  saved_plans_to_homes , saved_homes_to_bin ]
    loaded_plans = [ loaded_home , loaded_pkg ]

    for i in range(len(saved_plans)):
        temp_keys = list(saved_plans[i].keys())
        for keys in temp_keys:
            temp_keys2 = list(saved_plans[i][keys].keys())
            for keys2 in temp_keys2:
                file_name = saved_plans[i][keys][keys2]
                file_path = arg_file_path + file_name
                with open(file_path, 'r') as file_open:
                    loaded_plan = yaml.load(file_open)
                    loaded_plans[i][keys][keys2] = loaded_plan

    rospy.loginfo("Loaded the plans....")
    return loaded_plans
        


def main():

    rospy.init_node('ur5_2_place', anonymous=True)

    loaded_plans = Initialise_plan()
    ur5_2 = Ur5Moveit('ur5_2')
    Do = DispatchedOrder()
    Do.get_dispatched()

    priority_color = { 'HP':'red' ,
                       'MP':'yellow' ,
                       'LP':'green'}
    order_shipped = 0  
    Do.wait_to_dispatched()
    Do.logical_camera()

    current_shipped_position = 'zero'
    Do._going_position = 'zero'

    while not rospy.is_shutdown():

        if order_shipped < Do._no_of_dispatched and len(Do._dispatched_list) > 0 :

            current_order = Do._dispatched_list[0]
            Current_details = Do._dispatched_order[current_order]
            Do._shipped_time[current_order] = rospy.get_time()
            Do._going_position = current_shipped_position
            time_elapsed = Do._dispatched_time[current_order] + 6.7 - Do._shipped_time[current_order]
            home_position = prefered_home_position( current_shipped_position , Current_details["Priority"] , time_elapsed )

            ur5_2.hard_play_saved_trajectory(loaded_plans[0][current_shipped_position ][home_position],3)
            current_shipped_position = home_position
            Do.wait_package_to_arrived()
            rospy.sleep(0.1)    
            ur5_2.attach_box_in_gazebo()
            ur5_2.hard_play_saved_trajectory(loaded_plans[1][current_shipped_position][priority_color[Current_details["Priority"]]],3)
            ur5_2.detach_box_in_gazebo()

            order_shipped = order_shipped +1
            current_shipped_position = priority_color[Current_details["Priority"]]
            updated_the_shipped_sheet( Current_details , order_shipped)

            del Do._dispatched_order[Do._dispatched_list[0]]
            del Do._dispatched_time[Do._dispatched_list[0]]
            del Do._shipped_time[Do._dispatched_list[0]]
            del Do._dispatched_list[0]

        else:
            continue


if __name__ == '__main__':

    main()
    

    
    




