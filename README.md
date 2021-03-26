eyantra(2020-21)

Team VB#0574

Theme "Vargi-Bots"

BY - Raj kumar Gupta
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
This is an alternative solution to eyantra 'TASK-5/6'. Our team have sumbitted the best solution possible to the problem statement. I have done this solution on my own its not best optimistic solution but its still good to work. When all models in Gazebo Environment are properly spwaned, Packages on the shelf are identified on the shelf using image processing , then Inventory sheet is update through Iot which is implemented by creating a bridge between Ros and Iot through a python script. After this the saved trajectories are loaded on the node ur5_1_pick and correspondingly on ur5_2_place as loading the trajectories takes time therefore it is loaded in the start. Then Gazebo is ready to recieved order , as soon as order is recieved the  node incoming_order_handler update the Incoming Order sheet . As the ur5_1 process the incoming order according to their priority after dispatching it on conveyor belt , Order Dispatch sheet is updated and information about diaptched order is send to ur5_2 .Email is sent to customer about the Dispatch of order.When ur5_2 recieved the dispatched infromation it process according to its current position and decide which home position on belt it prefer. As package arrived near ur5_2 it stop at prefered position, ur5_2 then process this order in its correspondiong bin and email is sent to customer after shipping the order. A real-time Dashboard is implemented to visualise the performance of Warehouse.
