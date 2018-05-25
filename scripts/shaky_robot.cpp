#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt8.h"
#include <fstream>
#include <iostream>

ros::Subscriber robot_state_sub, robot_command_sub, pos_sub;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point rposcom, rposstate, apos;
geometry_msgs::Quaternion q;

std_msgs::UInt8 vibInt;

std::ofstream armf ("armz_command_position.txt");
std::ofstream commandf ("robotz_command_position.txt");
std::ofstream robotfy ("roboty_state_position.txt");
std::ofstream robotf ("robotz_state_position.txt");

bool stateUp, comUp, armUp, init = false;
ros::Time t;
double dt;

void initNode(){
    t = ros::Time::now();
}

void down(){
    armf.close();
    robotf.close();
    robotfy.close();
    commandf.close();
}

void print (){
    dt = dt + ros::Time::now().toSec() - t.toSec();
    t = ros::Time::now();

    armf << apos.z << " " << dt << std::endl;
    robotf << rposstate.z << " " << dt << std::endl;
    commandf << rposcom.z << " " << dt << std::endl;
    robotfy << rposstate.y << " " << dt << std::endl;

    stateUp = false;
    armUp = false;
    comUp = false;
}

void robotcommand (const geometry_msgs::PoseStamped::ConstPtr& msg){
    rposcom.z = msg->pose.position.z;
    comUp = true;
    if (armUp && stateUp){
        print();
    }   
}

void robot (const geometry_msgs::PoseStamped::ConstPtr& msg){
    rposstate.z = msg->pose.position.z;
    rposstate.y = msg->pose.position.y;
    stateUp = true;
    if (armUp && comUp){
        print();
    }   
}

void camera (const geometry_msgs::Point::ConstPtr& msg){
    apos.z = msg->z;
    armUp = true;
    if (stateUp && comUp){
        print();
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "shaky_robot_test");
    ros::NodeHandle n;
    
    if (!init){
        initNode();
        init = true;
    }

    robot_command_sub = n.subscribe("/iiwa/command/CartesianPose", 1, robotcommand);  
    robot_state_sub = n.subscribe("/iiwa/state/CartesianPose", 1, robot);  
    // rostopic echo /iiwa/command/CartesianPose
    pos_sub = n.subscribe("/camera/rightHipHand", 1, camera);      
    // rostopic echo /camera/rightHipHand

    if (!ros::ok){
        down();
    }

    ros::spin();

    return 0;
}
