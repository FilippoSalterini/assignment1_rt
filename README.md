# Assignment1 - Research Track 1

The assignment involves spawning two turtles and, using two nodes written in C++, controlling the turtles with commands while ensuring they do not collide with each other and stopping the moving turtle if its position is too close to the boundaries.

## Overview
This project involves the development of two c++ nodes:
1. **node 1** called **Ui**: this node spawn a new turtle in the environment and give a table of command to control the 2 turtles.
2. **node 2** called **Distance**: this node checks the relative distance between turtle1 and turtle2, stops the moving turtle if the two turtles are “too close” and stops the moving turtle if the position is too close to the boundaries.

## Nodes

### UI nodes description
This node controls two turtles in a ROS environment (Turtlesim). It allows user input to Spawn a second turtle and Command each turtle to move in specific directions (forward, backward, left, right, or diagonally) at a specified speed.
* File name: ui.cpp
* Key Features:
I define 2 publisher ```Pub1``` and ```Pub2``` needed later to publish my linear velocities.  
The function ```void TurtleControl(const string& turtle_name, const string& direction, float speed)```  control the turtle, taking as input turtle_name, direction and speed. Inside the function i initializes a ```geometry_msgs::Twist``` object ```(my_vel)``` to set linear velocities ```linear.x``` and ```linear.y``` based on the given direction.  
Then publish velocities to the correct turtle ```pub1.publish(my_vel)``` for turtle1 and ```pub2.publish(my_vel)``` for turtle2.
Sleep for 1 second to execute the movement ```ros::Duration(1.0).sleep()``` and then to stop the turtle after 1 sec publish zero velocity message ```pub1.publish(geometry_msgs::Twist())```.
In the main :
    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn")
