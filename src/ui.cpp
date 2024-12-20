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
    //the directions of the turtles are oriented in respect of their head 
    if (direction == "f") {
    my_vel.linear.x = speed; // I gave a positive number to go forward
    my_vel.linear.y = 0;
    }
    else if(direction == "b") {
    my_vel.linear.x = -speed; //I take the speed and put it negative to go backward
    my_vel.linear.y = 0;
    }
    else if(direction == "l") {
    my_vel.linear.y = speed; 
    my_vel.linear.x = 0;
    }
    else if(direction == "r") {
    my_vel.linear.y = -speed; 
    my_vel.linear.x = 0;
    } 
    // i implemented later also the commando to go diagonally
    else if (direction == "rf") {
    my_vel.linear.x = speed;
    my_vel.linear.y = -speed; 
    }
    else if(direction == "lf") {
    my_vel.linear.x = speed; 
    my_vel.linear.y = speed;
    }
    else if(direction == "lb") {
    my_vel.linear.y = speed;
    my_vel.linear.x = -speed; 
    }
    else if(direction == "rb") {
    my_vel.linear.y = -speed;
    my_vel.linear.x = -speed; 
    }

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
    
    ros::Rate loop_rate(10); 
    //ciclo while
    while (ros::ok()) {
        string turtle_name, direction;
        float speed;
        
        cout<<"LIST OF MOVEMENT TO CONTROL THE TURTLES: \n"<<endl;
        cout<<" [FORWARD] digit [f]\n [BACKWARD] digit [b]\n";
        cout<<" [LEFT] digit [l]\n [RIGHT] digit [r]\n";  
        cout<<" [RIGHT-FORWARD] digit [rf]\n [LEFT-FORWARD] digit [lf]\n";  
        cout<<" [RIGHT-BACKWARD] digit [rb]\n [LEFT-BACKWARD] digit [lb]\n";  
        
        while(true) {
        cout << "Enter turtle name (turtle1 or turtle2): ";
        cin >> turtle_name;
        
        if(turtle_name == "turtle1" || turtle_name == "turtle2") {
        break; //valid turtle name so it can exit ftom the loop and procede with the direction
        } else {
        cout<<"Wrong turtle name! Insert turtle1 or turtle2: "<<endl;
             }
        }
        
        while(true) {
        cout << "Enter Direction: ";
        cin >> direction;
        
        if(direction == "f" || direction == "l" || direction == "b" || direction == "r" || direction == "rf" || direction == "lf" || direction == "rb" || direction == "lb") {
        break; //valid direction so it can exit from the loop and procede with the speed
                } else {
        cout<<"Wrong direction! Insert a correct direction: \n [FORWARD] digit [f] - [BACKWARD] digit [b] \n";
        cout<<" [LEFT] digit [l] - [RIGHT] digit [r]\n";  
        cout<<" [RIGHT-FORWARD] digit [rf] - [LEFT-FORWARD] digit [lf]\n";  
        cout<<" [RIGHT-BACKWARD] digit [rb] - [LEFT-BACKWARD] digit [lb]\n"; 
             }
        }
        
        while(true) {
        cout<<"Enter speed of the turtle: ";
        cin >> speed;
        
        if(speed >= 0) {
        break; //valid value for the speed so it can exit from the loop and recall the function turtlecontrol
        } else{
        cout<<"Insert a positive value for the speed!"<<endl;
            }
        }  
        
        TurtleControl(turtle_name, direction, speed);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
