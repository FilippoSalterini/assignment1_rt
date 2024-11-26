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

// i define 2 publisher for the velocity of the 2 turtle
ros::Publisher vel_turtle_pub1;
ros::Publisher vel_turtle_pub2;

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

void TurtleWall() {
      //i defined 4 const value that represent the walls, and a treshold for the limit distance between turtle and walls
       const float WallX = 10.0;
       const float WallY = 10.0;
       const float WallX0 = 1.0;
       const float WallY0 = 1.0;
       const float WALL_THRESHOLD = 0.1;
       
    geometry_msgs::Twist vel_turtle1;
    geometry_msgs::Twist vel_turtle2;
    
    if ((WallX - turtle1_position.x) < WALL_THRESHOLD) {
            ROS_WARN_STREAM("THE TURTLE1 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle1.linear.x = -0.5;
        vel_turtle_pub1.publish(vel_turtle1);
    } else if ((turtle1_position.x - WallX0) < WALL_THRESHOLD) {
            ROS_WARN_STREAM("THE TURTLE1 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle1.linear.x = 0.5;
        vel_turtle_pub1.publish(vel_turtle1);
    }

    if ((WallY - turtle1_position.y) < WALL_THRESHOLD) {
            ROS_WARN_STREAM("THE TURTLE1 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle1.linear.y = -0.5;
        vel_turtle_pub1.publish(vel_turtle1);
    } else if ((turtle1_position.y - WallY0) < WALL_THRESHOLD) {
            ROS_WARN_STREAM("THE TURTLE1 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle1.linear.y = 0.5;
        vel_turtle_pub1.publish(vel_turtle1);
    }

    if ((WallX - turtle2_position.x) < WALL_THRESHOLD) {
            ROS_WARN_STREAM("THE TURTLE2 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle2.linear.x = -0.5;
        vel_turtle_pub2.publish(vel_turtle2);
    } else if ((turtle2_position.x - WallX0) < WALL_THRESHOLD) {
                ROS_WARN_STREAM("THE TURTLE2 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle2.linear.x = 0.5;
        vel_turtle_pub2.publish(vel_turtle2);
    }

    if ((WallY - turtle2_position.y) < WALL_THRESHOLD) {
                ROS_WARN_STREAM("THE TURTLE2 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle2.linear.y = -0.5;
        vel_turtle_pub2.publish(vel_turtle2);
    } else if ((turtle2_position.y - WallY0) < WALL_THRESHOLD) {
                ROS_WARN_STREAM("THE TURTLE2 IS TOO CLOSE TO THE WALL!: \n");
        vel_turtle2.linear.y = 0.5;
        vel_turtle_pub2.publish(vel_turtle2);
    }
}

void TurtleImpact() {
       const float TRESHOlD = 1.0; //here I define a constant distance value for the 2 turtles to be used in the function to avoid the collision between the two
       //i set a float distance that gaves me in realt time the distances between the 2 turtles
       float distance = TurtleDistance(turtle1_position, turtle2_position);
       //create 2 twist msg for velocity adjustment
       geometry_msgs::Twist vel_turtle1;
       geometry_msgs::Twist vel_turtle2;
       //here I check if the distance between the two turtles is less than the previously defined limit "TRESHOLD"
       if(distance < TRESHOlD) {
       ROS_WARN_STREAM("THE TURTLE ARE TOO CLOSE!: \n" << distance);
       // Move turtles apart by steering them in opposite directions
       if(turtle1_position.x > turtle2_position.x){
       vel_turtle1.linear.x = 0.5;
       vel_turtle2.linear.x = -0.5;
       vel_turtle_pub1.publish(vel_turtle1);
       vel_turtle_pub2.publish(vel_turtle2);
       } else {
       vel_turtle1.linear.x = -0.5;
       vel_turtle2.linear.x = 0.5;
       vel_turtle_pub1.publish(vel_turtle1);
       vel_turtle_pub2.publish(vel_turtle2);
       }
       if(turtle1_position.y > turtle2_position.y){
       vel_turtle1.linear.y = 0.5;
       vel_turtle2.linear.y = -0.5;
       vel_turtle_pub1.publish(vel_turtle1);
       vel_turtle_pub2.publish(vel_turtle2);
       } else {
       vel_turtle1.linear.y = -0.5;
       vel_turtle2.linear.y = 0.5;
       vel_turtle_pub1.publish(vel_turtle1);
       vel_turtle_pub2.publish(vel_turtle2);
              }
       }
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "distance_between_turtle_and_wall");
	ros::NodeHandle nh;
        ros::Subscriber turtle1_sub = nh.subscribe("/turtle1/pose", 10, Turtle1CallBackPosition);
        ros::Subscriber turtle2_sub = nh.subscribe("/turtle2/pose", 10, Turtle2CallBackPosition);
        
        vel_turtle_pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        vel_turtle_pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
        
        ros::Rate loop_rate(10);
    //ciclo while
    while (ros::ok()) {
        
        //now i compute the distance between the 2 turtle
        float distance = TurtleDistance(turtle1_position, turtle2_position);
        
        ROS_INFO_STREAM("Distance between turtles: \n" << distance); //info to the monitor for checking the distance
        TurtleImpact();     //call the impact function to check if they are too close
        TurtleWall();       //function that permitt the turtles to avoid the walls
        ros::spinOnce();    //here process the turtle callback, without any tipe of blocking loops
        loop_rate.sleep();
    }

    return 0;
}
