#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
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

std::ofstream orif ("ori_drift_test.txt");

bool init = false;
ros::Time t;
double dt;
float x,y,z,w;

void initNode(){
    t = ros::Time::now();
}

void down(){
    orif.close();
}

void print (){
    dt = dt + ros::Time::now().toSec() - t.toSec();
    t = ros::Time::now();

    orif << x << " " << y << " " << z << " " << w << " " << dt << std::endl;
    ros::Duration(0.25).sleep();
}

void ori (const sensor_msgs::Imu::ConstPtr& msg){
    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;
    print();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "ori_drift_test");
    ros::NodeHandle n;
    
    if (!init){
        initNode();
        init = true;
    }

    pos_sub = n.subscribe("/myo_raw/myo_imu", 1, ori);      

    if (!ros::ok){
        down();
    }

    ros::spin();

    return 0;
}
