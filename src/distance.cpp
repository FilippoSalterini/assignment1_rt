#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <cmath>
#include <string>

using namespace std;

turtlesim::Pose turtle1_position;
turtlesim::Pose turtle2_position;

//now i have to callback the position for turtle1 and turtle2
void Turtle1CallBackPosition (const turtlesim::Pose::ConstPtr& msg){
      turtle1_position = *msg;
}

void Turtle2CallBackPosition (const turtlesim::Pose::ConstPtr& msg){
      turtle2_position = *msg;
}

float TurtleDistance (const turtlesim::Pose& position1, const turtlesim::Pose& position2) {
      //here i compute the effective math calculation for the distance
      return sqrt(pow(position1.x - position2.x, 2) + pow(position1.y - position2.y, 2)); 
}
int main(int argc, char **argv) {

	ros::init(argc, argv, "distance_between_turtle");  
	ros::NodeHandle nh;
        ros::Subscriber turtle1_sub = nh.subscribe("/turtle1/pose", 10, Turtle1CallBackPosition);
        ros::Subscriber turtle2_sub = nh.subscribe("/turtle2/pose", 10, Turtle2CallBackPosition);
        
        ros::Rate loop_rate(10);
        
    //ciclo while
    while (ros::ok()) {
        //now i compute the distance between the 2 turtle
        float distance = TurtleDistance(turtle1_position, turtle2_position);
        ROS_INFO_STREAM("Distance between turtles: " << distance); //info to the monitor for the distance
        ros::spinOnce(); // Process callbacks
        loop_rate.sleep();
    }

    return 0;
}
