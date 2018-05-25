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

ros::Publisher msg_pub, gripper_pub, vibrate;
ros::Subscriber right_pos_sub, left_pos_sub, gest_sub, calib_sub, arm_ori_sub, robot_ori_sub, left_hand_ctr, right_hand_ctr;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point p;
geometry_msgs::Quaternion q;

robotiq_c_model_control::CModel_robot_output gripper;
std_msgs::UInt8 vibInt;
tf::Quaternion q0a, q0r, qRelative, Qa;

bool calibrated, run, gripRegistered, armDown, armOriUpdated, robotOriUpdated, robotInit, gripperClosed, handOpen, rightHandCtr = false;

float REST_POS_X;    //arm coordinate when in "calibration position", x
float REST_POS_Z;    //arm coordinate when in "calibration position", z
float x_pos_old, y_pos_old, z_pos_old;
float rob_pos_x, rob_pos_y, rob_pos_z;

const float MIN_VALUE_Z = 0.34;        //Minimun value allowed in z-direction
const float ROBOT_GOAL_POS_X = 0.6;    //Coordinate for Calibration position in x
const float ROBOT_GOAL_POS_Z = 0.5;    //Coordinate for Calibration position in z

const float r = 0.4;            //inner cirkle
const float R = 0.90;           //outer cirkle

void initGripper(){
    gripper.rACT = 0; //reset gripper
    gripper_pub.publish(gripper);
    ros::Duration(1).sleep();
    gripper.rACT = 1; //activate gripper
    gripper.rGTO = 1;
    gripper.rSP  = 105;
    gripper.rFR  = 25;
    gripper_pub.publish(gripper);
}

void calibration(const geometry_msgs::PoseStamped::ConstPtr& msg){  
    //Init
    rightHandCtr = true;		//Has to start with right hand
    initGripper();
    //Input from calibration node
    REST_POS_X = msg->pose.position.x;
    REST_POS_Z = msg->pose.position.z;
    float QXold = msg->pose.orientation.x;
    float QYold = msg->pose.orientation.y;
    float QZold = msg->pose.orientation.z;
    float QWold = msg->pose.orientation.w;    
    q0a = tf::Quaternion(QZold, QYold, QXold, QWold);   //OBS bytat plats pÃ¥ x o z 
    vibInt.data = 1;
    vibrate.publish(vibInt);
    calibrated = true;
    ROS_INFO("Arm imitation with gripper ready");
}

void gripperControl (const ros_myo::MyoPose::ConstPtr& msg){
    if ((msg->pose == 2) && !gripperClosed){   //if gripp registered
        gripper.rPR = 255;     //close gripper   
        gripper_pub.publish(gripper);
        gripperClosed = true;
        handOpen = false;
    }
    else if (handOpen && gripperClosed && (msg->pose == 5) ){
        gripper.rPR = 0;      //open gripper 
        gripper_pub.publish(gripper);
        gripperClosed = false;
    }
    else if (!handOpen && (msg->pose == 1) ){
        handOpen = true;
    }
    else if (rightHandCtr && msg->pose == 3){ 	//change control to left hand
        rightHandCtr = false;
        vibInt.data = 2;
        vibrate.publish(vibInt);
        ROS_INFO("Left hand control");

    }
    else if (!rightHandCtr && msg->pose == 4){	//change control to right hand
        rightHandCtr = true;
        vibInt.data = 2;
        vibrate.publish(vibInt);
        ROS_INFO("Right hand control");
    }
}

void msgControl (const geometry_msgs::Point::ConstPtr& msg){
    
    if ((msg->z > 0) && !run && armDown){        //if hand above shoulder
        run = true;
        armDown = false;
        vibInt.data = 1;
        vibrate.publish(vibInt);
        ROS_INFO("Control ON");
    }
    else if ((msg->z > 0) && run && armDown){
        run = false;
        armDown = false;
        vibInt.data = 1;
        vibrate.publish(vibInt);
        ROS_INFO("Control OFF");
    }
    else if ((msg->z < 0)){
        armDown = true;
    }
}

void rightMsgControl (const geometry_msgs::Point::ConstPtr& msg){
    if (!rightHandCtr){	//if left hand control
        msgControl(msg);
    }
}

void leftMsgControl (const geometry_msgs::Point::ConstPtr& msg){
    if (rightHandCtr){
        msgControl(msg);
    }
}

void armOriControl (const sensor_msgs::Imu::ConstPtr& msg){
    if (calibrated && run && robotOriUpdated) {         
        // Update quaternions
        float QXnew = msg->orientation.x;
        float QYnew = msg->orientation.y;
        float QZnew = msg->orientation.z;
        float QWnew = msg->orientation.w;
        
        Qa = tf::Quaternion(QZnew, QYnew, QXnew, QWnew);   //OBS bytat plats pÃ¥ x o z
        
        armOriUpdated = true;
        //ROS_INFO("Arm oritentation updated");
    }
}

void robotOriControl (const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (calibrated){
        if (!robotInit){        // compute orientation for the relative quaternion vector, qRelative
            float rQXold = msg->pose.orientation.x;
            float rQYold = msg->pose.orientation.y;
            float rQZold = msg->pose.orientation.z;
            float rQWold = msg->pose.orientation.w;

            q0r = tf::Quaternion(rQXold, rQYold, rQZold, rQWold);
            qRelative = q0a.inverse()*q0r;      // Qa * qRelative = Qr;
            robotInit = true;
        }

        rob_pos_x = msg->pose.position.x;
        rob_pos_y = msg->pose.position.y;
        rob_pos_z = msg->pose.position.z;

        robotOriUpdated = true;
    }
}

void makeMsg (const geometry_msgs::Point::ConstPtr& msg){
  
    if (calibrated && run && armOriUpdated){  // && robotOriUpdated
        float x_pos, y_pos, z_pos;

        //x-position        
        float Ax = msg->x / REST_POS_X;
        x_pos = ROBOT_GOAL_POS_X * sqrt(Ax);
        x_pos = fmax(0, fmin(R, x_pos));     // 0 = min value x
        
        //y-position
        y_pos = fmin(R, fabs(msg->y));
        y_pos = copysignf(y_pos, msg->y);

        //z-position
        float Az = msg->z / REST_POS_Z;
        z_pos = ROBOT_GOAL_POS_Z * sqrt(Az);      
        z_pos = fmax(MIN_VALUE_Z, fmin(R, z_pos));

        float vLength = sqrt(x_pos*x_pos + y_pos*y_pos + (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));

        if (vLength > R){
            p.x = x_pos_old;
            p.y = y_pos_old;
            p.z = z_pos_old;            
            //p.x = sqrt(R*R - y_pos*y_pos - (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));        // R = ~0.9
            //p.y = y_pos;
            //p.z = z_pos;
        }
        else if (vLength < r){
            p.x = x_pos_old;
            p.y = y_pos_old;
            p.z = z_pos_old;
            //p.x = sqrt(r*r - y_pos*y_pos - (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));        // r = 0.4
            //p.y = y_pos;
            //p.z = z_pos;
        }
        else{
            p.x = x_pos;
            p.y = y_pos;
            p.z = z_pos;
            x_pos_old = x_pos;
            y_pos_old = y_pos;
            z_pos_old = z_pos;
        }
    } 
    else if (calibrated && !run){
        p.x = rob_pos_x;
        p.y = rob_pos_y;
        p.z = rob_pos_z;
    }

    //Quaternion
    tf::Quaternion Qfinal = Qa.operator*=(qRelative);
    //q.x = (float) Qfinal.getAxis().x();
    //q.y = (float) Qfinal.getAxis().y(); 
    //q.z = (float) Qfinal.getAxis().z(); 
    //q.w = Qfinal.getW(); 

    q.x = 1;
    q.y = 0.711;
    q.z = 1;
    q.w = 0.703; 

    //Create msg
    pose.position = p;
    pose.orientation = q;
    my_pos.pose = pose;
    my_pos.header.stamp = ros::Time::now();
    my_pos.header.frame_id = "iiwa_imitate_grip_command";

    msg_pub.publish(my_pos);
    armOriUpdated = false;
    robotOriUpdated = false;
}

void rightHandPos (const geometry_msgs::Point::ConstPtr& msg){
    if (rightHandCtr){
        makeMsg(msg);
    }
}

void leftHandPos (const geometry_msgs::Point::ConstPtr& msg){
    if (!rightHandCtr){
        makeMsg(msg);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "arm_imitation_gripper");
    ros::NodeHandle n;

    calib_sub = n.subscribe("/arm_imitation/Calibration", 1, calibration);
    left_hand_ctr = n.subscribe("/camera/leftHandShoulder", 1, leftMsgControl);     // rostopic echo /camera/leftHandShoulder
    right_hand_ctr = n.subscribe("/camera/rightHandShoulder", 1, rightMsgControl);     // rostopic echo /camera/rightHandShoulder
    arm_ori_sub = n.subscribe("/myo_raw/myo_imu", 1, armOriControl);
    robot_ori_sub = n.subscribe("/iiwa/state/CartesianPose", 1, robotOriControl);
    gest_sub = n.subscribe("/myo_raw/myo_gest", 1, gripperControl);
    right_pos_sub = n.subscribe("/camera/rightHipHand", 1, rightHandPos);
    left_pos_sub = n.subscribe("/camera/leftHipHand", 1, leftHandPos);

    msg_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1);
    gripper_pub = n.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 1);
    vibrate = n.advertise<std_msgs::UInt8>("/myo_raw/vibrate", 1);     

    ros::spin();

    return 0;
}

