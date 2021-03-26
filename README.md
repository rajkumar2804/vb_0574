eyantra(2020-21)

Team VB#0574

Theme "Vargi-Bots"

BY - Raj kumar Gupta
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
This is an alternative solution to eyantra 'TASK-5/6'. Our team have submitted the best solution possible to the problem statement. I have done this solution on my own it's not the best optimistic solution but its still good to work. When all models in Gazebo Environment are properly spawned, packages on the shelf are identified on the shelf using image processing, then the Inventory sheet is update through IoT which is implemented by creating a bridge between ROS and IoT through a python script. After this the saved trajectories are loaded on the node ur5_1_pick and correspondingly on ur5_2_place as loading the trajectories takes time therefore it is loaded in the start. Then Gazebo is ready to received order, as soon as order is received the  node incoming_order_handler update the Incoming Order sheet. As the ur5_1 process the incoming order according to their priority after dispatching it on conveyor belt, Order Dispatch sheet is updated and information about the dispatched order is sent to ur5_2. Email is sent to customer about the Dispatch of order. When ur5_2 received the dispatched information it processes according to its current position and decide which home position on belt it prefer. As the package arrived near ur5_2 it stops at preferred position, ur5_2 then process this order in its corresponding bin and email is sent to the customer after shipping the order. A real-time Dashboard is implemented to visualize the performance of Warehouse.

Warehouse

When all the models and packages are properly spawned. This is what the Gazebo environment look like:
![Screenshot from 2021-03-26 12-35-32](https://user-images.githubusercontent.com/25104480/112604621-5e86b300-8e3c-11eb-8b54-a4d94f77908f.png)
