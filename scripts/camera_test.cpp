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

std::ofstream cameraf ("camera_drift_z.txt");

bool init = false;
ros::Time t;
double dt;

void initNode(){
    t = ros::Time::now();
}

void down(){
    cameraf.close();
}

void print (){
    dt = dt + ros::Time::now().toSec() - t.toSec();
    t = ros::Time::now();
    cameraf << apos.z << " " << dt << std::endl;
}

void camera (const geometry_msgs::Point::ConstPtr& msg){
    apos.z = msg->z;
    print();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "camera_test");
    ros::NodeHandle n;
    
    if (!init){
        initNode();
        init = true;
    }

    pos_sub = n.subscribe("/camera/rightHipHand", 1, camera);      
    // rostopic echo /camera/rightHipHand

    if (!ros::ok){
        down();
    }

    ros::spin();

    return 0;
}
