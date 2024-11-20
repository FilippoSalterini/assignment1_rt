#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <string>

using namespace std;

ros::Publisher pub1;
ros::Publisher pub2;
  
void TurtleControl(const string& turtle_name, const string& direction, float speed) {
    geometry_msgs::Twist my_vel;
    
    //ricontrolla codice per la velocita positiva
    
    if (direction == "forward" && speed > 0) {
    my_vel.linear.x = speed; // i gave a positive number to go forward
    }
    else if(direction == "backward" && speed > 0) {
    my_vel.linear.x = -speed; //i take the speed and put it negative to go backward
    }
    else if(direction == "left" && speed > 0) {
    my_vel.linear.y = speed; //same concept before
    }
    else if(direction == "right" && speed > 0) {
    my_vel.linear.y = -speed; 
    } else {
    cout<<"Invalid direction"<<endl;
    
    return;
    } 
    
    // bordo 11,8

    if (turtle_name == "turtle1") {
        pub1.publish(my_vel);
        
        ros::Duration(1.0).sleep(); // Sleep for 1 second

        // Publish an empty message to stop Turtle1 (velocity = 0)
        pub1.publish(geometry_msgs::Twist());

    } 
    else if (turtle_name == "turtle2") {
        pub2.publish(my_vel);

        ros::Duration(1.0).sleep(); // Sleep for 1 second

        // Publish an empty message to stop Turtle2 (velocity = 0)
        pub2.publish(geometry_msgs::Twist());

    } 
    else {
        cout << "Turtle Name is not correct" << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ui");  
    ros::NodeHandle nh;

    // Initialize publishers
    pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Spawn the second turtle
    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv1;
    srv1.request.x = 2.0;  
    srv1.request.y = 2.0;
    srv1.request.theta = 0.0;
    srv1.request.name = "turtle2";
    client1.waitForExistence();
    client1.call(srv1);
    
    //ciclo while
    while (ros::ok()) {
        string turtle_name, direction;
        float speed;

        cout << "Enter turtle name (turtle1 or turtle2): ";
        cin >> turtle_name;

        cout << "Enter Direction [Forward,Backward,Left,Right]: ";
        cin >> direction;

        cout<<"Enter speed of the turtle (positive value): ";
        cin >> speed;
        
        TurtleControl(turtle_name, direction, speed);
        }
        
        return 0;
    }
