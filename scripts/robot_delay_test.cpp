#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt8.h"
#include <fstream>
#include <iostream>

ros::Publisher msg_pub, vibrate;
ros::Subscriber pos_sub, gest_sub, calib_sub, arm_ori_sub, robot_ori_sub;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point rpos, apos;
geometry_msgs::Quaternion q;

std_msgs::UInt8 vibInt;

std::ofstream armf ("arm_position.txt");
std::ofstream robotf ("robot_position.txt");

bool robotUp, armUp, init = false;
ros::Time t;
double dt;

void initNode(){
    t = ros::Time::now();
}

void down(){
    armf.close();
    robotf.close();
}

void print (){
    dt = dt + ros::Time::now().toSec() - t.toSec();
    t = ros::Time::now();

    armf << apos.y << " " << dt << std::endl;
    robotf << rpos.y << " " << dt << std::endl;

    robotUp = false;
    armUp = false;
}

void robot (const geometry_msgs::PoseStamped::ConstPtr& msg){
    rpos.y = msg->pose.position.y;
    robotUp = true;
    if (armUp){
        print();
    }   
}

void camera (const geometry_msgs::Point::ConstPtr& msg){
    apos.y = msg->y;
    armUp = true;
    if (robotUp){
        print();
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "robot_delay_test");
    ros::NodeHandle n;
    
    if (!init){
        initNode();
        init = true;
    }

    robot_ori_sub = n.subscribe("/iiwa/state/CartesianPose", 1, robot);  
    // rostopic echo /iiwa/state/CartesianPose
    pos_sub = n.subscribe("/camera/rightHipHand", 1, camera);      
    // rostopic echo /camera/rightHipHand

    if (!ros::ok){
        down();
    }

    ros::spin();

    return 0;
}
