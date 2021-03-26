eyantra(2020-21)

Team VB#0574

Theme "Vargi-Bots"

BY - Raj kumar Gupta
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
This is an alternative solution to eyantra 'TASK-5/6'. Our team have submitted the best solution possible to the problem statement. I have done this solution on my own it's not the best optimistic solution but its still good to work. When all models in Gazebo Environment are properly spawned, packages on the shelf are identified on the shelf using image processing, then the Inventory sheet is update through IoT which is implemented by creating a bridge between ROS and IoT through a python script. After this the saved trajectories are loaded on the node ur5_1_pick and correspondingly on ur5_2_place as loading the trajectories takes time therefore it is loaded in the start. Then Gazebo is ready to received order, as soon as order is received the  node incoming_order_handler update the Incoming Order sheet. As the ur5_1 process the incoming order according to their priority after dispatching it on conveyor belt, Order Dispatch sheet is updated and information about the dispatched order is sent to ur5_2. Email is sent to customer about the Dispatch of order. When ur5_2 received the dispatched information it processes according to its current position and decide which home position on belt it prefer. As the package arrived near ur5_2 it stops at preferred position, ur5_2 then process this order in its corresponding bin and email is sent to the customer after shipping the order. A real-time Dashboard is implemented to visualize the performance of Warehouse.

Warehouse
--------------------------------------------------------------------------------------------------------------------------------------------------------------------
When all the models and packages are properly spawned. This is what the Gazebo environment look like:
![Screenshot from 2021-03-26 12-35-32](https://user-images.githubusercontent.com/25104480/112604621-5e86b300-8e3c-11eb-8b54-a4d94f77908f.png)

UR5#1 Arm
--------------------------------------------------------------------------------------------------------------------------------------------------------------------
There is  a general home position for the UR5#1 arm which look like this :

UR5#1 arm home position:
![ur5_1_home](https://user-images.githubusercontent.com/25104480/112609153-30f03880-8e41-11eb-9b44-c78d6e1aa6b7.png)

UR5#1 Arm
------------------------------------------------------------------------------------------------------------------------------------------------------------------
There are three home position for UR5#2 arm:- home#1 , home#2 and home#3. UR5#2 arm choose the home position to go on the basis of the dispatched order and time elapsed between a consecutive shipping and dispatch.The home positions look like this:

UR5#2 arm home position:

home#1
![ur5_2_home1](https://user-images.githubusercontent.com/25104480/112610576-dce65380-8e42-11eb-9484-2cd1fbcac96c.png)

home#2
![ur5_2_home2](https://user-images.githubusercontent.com/25104480/112611185-83325900-8e43-11eb-8a5b-6b89e904c95b.png)

home#3
![ur5_2_home3](https://user-images.githubusercontent.com/25104480/112611303-a957f900-8e43-11eb-9583-f9526647aa02.png)

Dashboard
------------------------------------------------------------------------------------------------------------------------------------------------------------------
A dashboard is implemented to visualise the performance of Warehouse System. The Dashboard look like as follow:
![Screenshot from 2021-03-26 15-23-47](https://user-images.githubusercontent.com/25104480/112615280-4ddc3a00-8e48-11eb-83ce-6d07c7a5fe9c.png)




