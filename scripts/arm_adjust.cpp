#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "std_msgs/UInt8.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "ros_myo/MyoPose.h" 
#include "math.h"
#include "robotiq_c_model_control/CModel_robot_output.h"

ros::Publisher msg_pub, vibrate, gripper_pub;
ros::Subscriber pos_sub, gest_sub, calib_sub, arm_ori_sub, robot_ori_sub, left_hand_sub;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point p;
geometry_msgs::Quaternion q;

std_msgs::UInt8 vibInt;
robotiq_c_model_control::CModel_robot_output gripper;

bool calibrated, gripRegistered, firstGrip, armOriUpdated, robotUpdated, robotInit, runOri, armDown, firstGripInit = false;
bool maxReached, minReached = false;

float REST_POS_X;    //Distance from hip to hand in z when arm is in "rest position"
float REST_POS_Z;    //Distance from hip to hand in x when arm is in "rest position"
float QXold, QYold, QZold, QWold, QXnew, QYnew, QZnew, QWnew, QXd, QYd, QZd, QWd;       //Quaternions for users arm (old, new, difference)
float rQXold, rQYold, rQZold, rQWold, rQXnew, rQYnew, rQZnew, rQWnew, rQXd, rQYd, rQZd, rQWd;     // Quaternions for robot (old, new, difference)
float posXnew, posYnew, posZnew, posXold, posYold, posZold, posXd, posYd, posZd;    //Positions for users arm (old, new, difference)
float rPosXnew, rPosYnew, rPosZnew, rPosXold, rPosYold, rPosZold, rPosXd, rPosYd, rPosZd;   //Positions for robot (old, new, difference)
float x_pos_old, y_pos_old, z_pos_old;
float rConstPosX, rConstPosY, rConstPosZ;
float rQXConst, rQYConst, rQZConst, rQWConst;

const float MIN_VALUE_Z = 0.34;    //Minimun value allowed in z-direction. Safety to not hit table etc.
const float ROBOT_GOAL_POS_X = 0.6;    //The position we want the robot to have in x-direction when in "rest position"
const float ROBOT_GOAL_POS_Z = 0.5;    //The position we want the robot to have in z-direction when in "rest position"
const float r = 0.4;
const float R = 0.90;

#define PI 3.14159265

void calibration(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    REST_POS_X = msg->pose.position.x;
    REST_POS_Z = msg->pose.position.z;
    QXold = msg->pose.orientation.x;
    QYold = msg->pose.orientation.y;
    QZold = msg->pose.orientation.z;
    QWold = msg->pose.orientation.w;
    //Init
    QXd, QYd, QZd, QWd = 0;
    posXd, posYd, posZd = 0;
    calibrated = true;
    ROS_INFO("Arm adjust ready");
    vibInt.data = 1;
    vibrate.publish(vibInt);
}

void msgControl (const ros_myo::MyoPose::ConstPtr& msg){
    if (calibrated){
        if ((msg->pose == 2) && !gripRegistered){
            gripRegistered = true;
            firstGrip = true;
        }
        else if ((msg->pose == 2) && firstGripInit){
            firstGripInit = false;
        }
        else{
            gripRegistered = false;
        }
    }
}

void oriControl (const geometry_msgs::Point::ConstPtr& msg){
    
    if ((msg->z > 0) && !runOri && armDown){        //if left hand above shoulder
        runOri = true;
        armDown = false;
        vibInt.data = 1;
        vibrate.publish(vibInt);
    }
    else if ((msg->z > 0) && runOri && armDown){
        runOri = false;
        armDown = false;
        vibInt.data = 1;
        vibrate.publish(vibInt);
    }
    else if (msg->z < 0){
        armDown = true;
    }
}

void armOriControl (const sensor_msgs::Imu::ConstPtr& msg){
    if (gripRegistered && robotUpdated) {
        if (firstGrip){
            QXold = msg->orientation.x;
            QYold = msg->orientation.y;
            QZold = msg->orientation.z;
            QWold = msg->orientation.w;
        }
        // Update quaternions
        QXnew = msg->orientation.x;
        QYnew = msg->orientation.y;
        QZnew = msg->orientation.z;
        QWnew = msg->orientation.w;
        
        QXd = QXd + (QXnew - QXold) - rQXd; 
        QYd = QYd + (QYnew - QYold) - rQYd; 
        QZd = QZd + (QZnew - QZold) - rQZd; 
        QWd = QWd + (QWnew - QWold) - rQWd;

        QXold = QXnew;
        QYold = QYnew;
        QZold = QZnew;
        QWold = QWnew;
        
        armOriUpdated = true;
        //ROS_INFO("Arm oritentation updated");

    }
}

void robotControl (const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (!robotInit){
        rQXold = msg->pose.orientation.x;
        rQYold = msg->pose.orientation.y;
        rQZold = msg->pose.orientation.z;
        rQWold = msg->pose.orientation.w;
        rPosXold = msg->pose.position.x;
        rPosYold = msg->pose.position.y;
        rPosZold = msg->pose.position.z;
        robotInit = true;
    }
    // Update quaternions
    rQXnew = msg->pose.orientation.x;
    rQYnew = msg->pose.orientation.y;
    rQZnew = msg->pose.orientation.z;
    rQWnew = msg->pose.orientation.w;

    rQXd = rQXnew - rQXold;   
    rQYd = rQYnew - rQYold;   
    rQZd = rQZnew - rQZold;   
    rQWd = rQWnew - rQWold;   

    rQXold = rQXnew;
    rQYold = rQYnew;
    rQZold = rQZnew;
    rQWold = rQWnew;

    // Update positions
    rPosXnew = msg->pose.position.x;
    rPosYnew = msg->pose.position.y;
    rPosZnew = msg->pose.position.z;

    rPosXd = (rPosXnew - rPosXold);
    rPosYd = (rPosYnew - rPosYold);
    rPosZd = (rPosZnew - rPosZold);

    rPosXold = rPosXnew;
    rPosYold = rPosYnew;
    rPosZold = rPosZnew;

    robotUpdated = true;
}

void makeMsg (const geometry_msgs::Point::ConstPtr& msg){
  
    if (armOriUpdated){     
        float x_pos, y_pos, z_pos;
        if (!runOri){
            if (firstGrip){
                posXold = msg->x;
                posYold = msg->y;
                posZold = msg->z;
                posXd = 0; 
                posYd = 0; 
                posZd = 0; 
                rPosXd = 0; 
                rPosYd = 0; 
                rPosZd = 0;
                rConstPosX = rPosXold;
                rConstPosY = rPosYold;
                rConstPosZ = rPosZold;
                firstGrip = false;
                firstGripInit = true;
            }
            //x-position
            posXnew = msg->x; 
            posXd = posXd + (posXnew - posXold) - rPosXd;
            posXold = posXnew;
            x_pos = rConstPosX + posXd;
            x_pos = fmax(0, x_pos);     // 0 = min value x
            
            //y-position
            posYnew = msg->y;
            posYd = posYd + (posYnew - posYold) - rPosYd;
            posYold = posYnew;
            y_pos = rConstPosY + posYd;
          
            //z-position
            posZnew = msg->z; 
            posZd = posZd + (posZnew - posZold) - rPosZd;
            posZold = posZnew;
            z_pos = rConstPosZ + posZd;
            if (maxReached && (z_pos - z_pos_old > 0)){
                z_pos = z_pos_old;
            }
            if (minReached && (z_pos - z_pos_old < 0)){
                z_pos = z_pos_old;
            }
            z_pos = fmax(0.34, z_pos);     // 0.34 = min value z

            float vLength = sqrt(x_pos*x_pos + y_pos*y_pos + (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));

            if (vLength > R){
                p.y = y_pos;
                p.z = z_pos;
                p.x = sqrt(R*R - y_pos*y_pos - (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));        // R = 0.8 evt 1
                maxReached = true;
            }
            else if (vLength < r){
                p.y = y_pos;
                p.z = z_pos;
                p.x = sqrt(r*r - y_pos*y_pos - (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));        // r = 0.4
                minReached = true;
            }
            else{
                p.x = x_pos;
                p.y = y_pos;
                p.z = z_pos;
                x_pos_old = x_pos;
                y_pos_old = y_pos;
                z_pos_old = z_pos;
                maxReached = false;
                minReached = false;
            }
            //Quaternion
            //q.x = rQXold; 
            //q.y = rQYold; 
            //q.z = rQZold; 
            //q.w = rQWold; 

            q.x = 1;
            q.y = 0.711;
            q.z = 1;
            q.w = 0.703; 
        }
        else{
             if (firstGrip){
                rConstPosX = rPosXold;
                rConstPosY = rPosYold;
                rConstPosZ = rPosZold;
                rQXConst = rQXold;
                rQYConst = rQYold;
                rQZConst = rQZold;
                rQWConst = rQWold;
                QXd = 0;
                QYd = 0;
                QZd = 0;
                QWd = 0;
                firstGrip = false;
                firstGripInit = true;
            }
            //Positions
            p.x = rConstPosX;
            p.y = rConstPosY;
            p.z = rConstPosZ;

            //Quaternion
            tf::Quaternion Q;
            Q = tf::Quaternion(rQXConst + QZd, rQYConst + QYd, rQZConst + QXd, rQWConst + QWd);       //obs Z to X and X to Z bc coordsystem
            Q.normalize();
            q.x = (float) Q.getAxis().x(); 
            q.y = (float) Q.getAxis().y(); 
            q.z = (float) Q.getAxis().z(); 
            q.w = (float) Q.getW(); 
        }
        //Create msg
        pose.position = p;
        pose.orientation = q;
        my_pos.pose = pose;
        my_pos.header.stamp = ros::Time::now();
        my_pos.header.frame_id = "iiwa_adjust_command";

        msg_pub.publish(my_pos);
        
        armOriUpdated = false;
        robotUpdated = false;
    } 
}


int main(int argc, char **argv){

    ros::init(argc, argv, "arm_adjust");
    ros::NodeHandle n;

    calib_sub = n.subscribe("/arm_imitation/Calibration", 1, calibration);      
    // rostopic echo /arm_imitation/Calibration
    gest_sub = n.subscribe("/myo_raw/myo_gest", 1, msgControl);      
    // rostopic echo /myo_raw/myo_gest
    arm_ori_sub = n.subscribe("/myo_raw/myo_imu", 1, armOriControl);      
    // rostopic echo /myo_raw/myo_imu
    robot_ori_sub = n.subscribe("/iiwa/state/CartesianPose", 1, robotControl);  
    // rostopic echo /iiwa/state/CartesianPose
    pos_sub = n.subscribe("/camera/rightHipHand", 1, makeMsg);      
    // rostopic echo /camera/rightHipHand
    left_hand_sub = n.subscribe("/camera/leftHandShoulder", 1, oriControl);

    msg_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1); 
    // rostopic echo /iiwa/command/CartesianPose
    gripper_pub = n.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 1);
    vibrate = n.advertise<std_msgs::UInt8>("/myo_raw/vibrate", 1); 

    ros::spin();

    return 0;
}
