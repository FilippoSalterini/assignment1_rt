# Assignment1 - Research Track 1

The assignment involves spawning two turtles and, using two nodes written in C++, controlling the turtles with commands while ensuring they do not collide with each other and stopping the moving turtle if its position is too close to the boundaries.

## Overview
This project involves the development of two c++ nodes:
1. **node 1** called **Ui**: this node spawn a new turtle in the environment and give a table of command to control the 2 turtles.
2. **node 2** called **Distance**: this node checks the relative distance between turtle1 and turtle2, stops the moving turtle if the two turtles are “too close” and stops the moving turtle if the position is too close to the boundaries.

## Nodes

### UI node description
This node controls two turtles in a ROS environment (Turtlesim). It allows user input to Spawn a second turtle and Command each turtle to move in specific directions (forward, backward, left, right, or diagonally) at a specified speed.
* File name: ui.cpp
* Key Features:
I define 2 publisher ```Pub1``` and ```Pub2``` needed later to publish my linear velocities.  
The function ```void TurtleControl(const string& turtle_name, const string& direction, float speed)```  control the turtle, taking as input turtle_name, direction and speed. Inside the function i initializes a ```geometry_msgs::Twist``` object ```(my_vel)``` to set linear velocities ```linear.x``` and ```linear.y``` based on the given direction.  
Then publish velocities to the correct turtle ```pub1.publish(my_vel)``` for turtle1 and ```pub2.publish(my_vel)``` for turtle2.
Sleep for 1 second to execute the movement ```ros::Duration(1.0).sleep()``` and then to stop the turtle after 1 sec publish zero velocity message ```pub1.publish(geometry_msgs::Twist())```.  
In the main :  
Creates a client ```ros::ServiceClient client1``` for the */spawn* service and spawn a turtle named turtle2 at position (2.0, 2.0). Runs continuously while ROS is active ```ros::ok())```, inside it will ask to the user the inputs : turtles name, direction and speed.
* Notes : To kill the node press *Ctrl+C*, the programm will ask you to insert the last data (turtle name, direction and speed) and the will shut down the node.

### DISTANCE node description
A node that checks the relative distance between turtle1 and turtle2, publish on a topic the distance, stops the moving turtle if the two turtles are “too close” and also stops the moving turtle if the position is too close to the boundaries
* File name: distance.cpp
* Key Features:
Declare 2 variables name ```turtle1_position``` and ```turtle2_position``` of the message type ```turtlesim::Pose``` that contains position and orientation data.  
Defined some function :
  1. *Callback functions* ```void Turtle1CallBackPosition (const turtlesim::Pose::ConstPtr& msg)``` and ```void Turtle2CallBackPosition (const turtlesim::Pose::ConstPtr& msg)``` necessary to update the position of the 2 turtles.
  2. ```float TurtleDistance (const turtlesim::Pose& position1, const turtlesim::Pose& position2)``` calculate the distance between the 2 turtles.  
  3. ```void TurtleWall()``` keep the turtles inside the "walls", defined constant values for the walls and a threshold, the function adjust the velocities ```linear.x``` or ```linear.y``` and then publish to the turtle the new values commands.
  4. ```void TurtleImpact()``` operate like ```void TurtleWall()``` defined a ```float distance = TurtleDistance(turtle1_position, turtle2_position)``` value that check in real time the position of the 2 turtles, and a const value named *THRESHOLD* equal to 1, when the distance of the 2 turtles is less than the threshold a warning is displayed and adjusts velocities to move the turtles in opposite directions along both x and y axes.  


Inside the main :  
```ros::Subscriber turtle1_sub = nh.subscribe("/turtle1/pose", 10, Turtle1CallBackPosition)``` and ```ros::Subscriber turtle2_sub = nh.subscribe("/turtle2/pose", 10, Turtle2CallBackPosition)``` subscribes to the /turtle2/pose and /turtle1/pose topic, which publishes the position and orientation of turtle1 and turtle2.
From code line *128* to *136* implemented a check-code for the position of the 2 turtles to be initialized.

## How to run it
1. Run the ```turtlesim_node``` using ```rosrun``` command to launch the Turtlesim simulation node
2. Run the ```ui``` node with ```rosrun``` command
3. Run the ```distance``` node with ```rosrun``` command
4. The interface managed by the ```ui``` node allows you to move the turtles via commands

## Notes
The nodes must be run in separate terminals and the ```ui``` node must be run before the ```distance``` node. In node ```ui``` to kill the node press *Ctrl+C*, the programm will ask you to insert the last data (turtle name, direction and speed) and the will shut down the node.  
For any further explanation check the commments inside the codes.






