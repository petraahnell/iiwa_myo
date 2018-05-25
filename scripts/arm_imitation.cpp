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

ros::Publisher msg_pub, vibrate;
ros::Subscriber pos_sub, gest_sub, calib_sub, arm_ori_sub, robot_ori_sub;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point p;
geometry_msgs::Quaternion q;

std_msgs::UInt8 vibInt;
tf::Quaternion q0a, q0r, qRelative, Qa, q0aInv, Qrot;

bool calibrated, gripRegistered, armOriUpdated, firstGrip, robotOriUpdated, robotInit = false;

float REST_POS_X;    //arm coordinate when in "calibration position", x
float REST_POS_Z;    //arm coordinate when in "calibration position", z
float x_pos_old, y_pos_old, z_pos_old;

const float MIN_VALUE_Z = 0.34;        //Minimun value allowed in z-direction
const float ROBOT_GOAL_POS_X = 0.6;    //Coordinate for Calibration position in x
const float ROBOT_GOAL_POS_Z = 0.5;    //Coordinate for Calibration position in z

const float r = 0.4;            //inner cirkle
const float R = 0.90;           //outer cirkle

void calibration(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //Input from calibration node
    REST_POS_X = msg->pose.position.x;
    REST_POS_Z = msg->pose.position.z;
    float QXold = msg->pose.orientation.x;
    float QYold = msg->pose.orientation.y;
    float QZold = msg->pose.orientation.z;
    float QWold = msg->pose.orientation.w;    
    q0a = tf::Quaternion(QZold, QYold, QXold, QWold);   //OBS bytat plats på x o z
    vibInt.data = 1;
    vibrate.publish(vibInt);        //vibrate myo
    calibrated = true;
    ROS_INFO("Arm imitation ready");
}

void msgControl (const ros_myo::MyoPose::ConstPtr& msg){
  
    if ((msg->pose == 2) && !gripRegistered){
        gripRegistered = true;
        firstGrip = true;
        vibInt.data = 1;
        vibrate.publish(vibInt);
        ROS_INFO("Control ON");
    }
    else if ((msg->pose == 2) && firstGrip){
        firstGrip = false;
    }
    else{
        gripRegistered = false;
        ROS_INFO("Control OFF");
    }
}

void armOriControl (const sensor_msgs::Imu::ConstPtr& msg){
    if (calibrated && gripRegistered && robotOriUpdated) {  // && robotOriUpdated
        
        // Update quaternions
        float QXnew = msg->orientation.x;
        float QYnew = msg->orientation.y;
        float QZnew = msg->orientation.z;
        float QWnew = msg->orientation.w;
        
        Qa = tf::Quaternion(QZnew, QYnew, QXnew, QWnew);   //OBS bytat plats på x o z
        
        armOriUpdated = true;
        //ROS_INFO("Arm oritentation updated");

    }
}

void robotOriControl (const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (calibrated){
        if (!robotInit){
            float rQXold = msg->pose.orientation.x;
            float rQYold = msg->pose.orientation.y;
            float rQZold = msg->pose.orientation.z;
            float rQWold = msg->pose.orientation.w;

            q0r = tf::Quaternion(rQXold, rQYold, rQZold, rQWold);
            qRelative = q0a.inverse()*q0r;      // Qa * qRelative = Qr;
            robotInit = true;
        }

        robotOriUpdated = true;
    }
}

void makeMsg (const geometry_msgs::Point::ConstPtr& msg){
  
    if (armOriUpdated){     
        float x_pos, y_pos, z_pos;

        //x-position
        float Ax = msg->x / REST_POS_X;
        x_pos = ROBOT_GOAL_POS_X * sqrt(Ax);
        x_pos = fmax(0, x_pos);     // 0 = min value x
        
        //y-position
        y_pos = msg->y;

        //z-position
        float Az = msg->z / REST_POS_Z;
        z_pos = ROBOT_GOAL_POS_Z * sqrt(Az);      
        z_pos = fmax(MIN_VALUE_Z, z_pos);

        float vLength = sqrt(x_pos*x_pos + y_pos*y_pos + (z_pos-MIN_VALUE_Z)*(z_pos-MIN_VALUE_Z));

        if (vLength < r || vLength > R){
            p.x = x_pos_old;
            p.y = y_pos_old;
            p.z = z_pos_old;
        }
        else{
            p.x = x_pos;
            p.y = y_pos;
            p.z = z_pos;
            x_pos_old = x_pos;
            y_pos_old = y_pos;
            z_pos_old = z_pos;
        }

        //Quaternion
        tf::Quaternion Qfinal = Qa.operator*=(qRelative);
        q.x = (float) Qfinal.getAxis().x(); 
        q.y = (float) Qfinal.getAxis().y(); 
        q.z = (float) Qfinal.getAxis().z(); 
        q.w = Qfinal.getW(); 

        //q.x = 1;
        //q.y = 0.711;
        //q.z = 1;
        //q.w = 0.703; 

        //Create msg
        pose.position = p;
        pose.orientation = q;
        my_pos.pose = pose;
        my_pos.header.stamp = ros::Time::now();
        my_pos.header.frame_id = "iiwa_imitation_command";

        msg_pub.publish(my_pos);
        armOriUpdated = false;
        robotOriUpdated = false;
    } 
}


int main(int argc, char **argv){

    ros::init(argc, argv, "arm_imitation");
    ros::NodeHandle n;

    calib_sub = n.subscribe("/arm_imitation/Calibration", 1, calibration);      
    // rostopic echo /arm_imitation/Calibration
    gest_sub = n.subscribe("/myo_raw/myo_gest", 1, msgControl);      
    // rostopic echo /myo_raw/myo_gest
    arm_ori_sub = n.subscribe("/myo_raw/myo_imu", 1, armOriControl);      
    // rostopic echo /myo_raw/myo_imu
    robot_ori_sub = n.subscribe("/iiwa/state/CartesianPose", 1, robotOriControl);  
    // rostopic echo /iiwa/state/CartesianPose
    pos_sub = n.subscribe("/camera/rightHipHand", 1, makeMsg);      
    // rostopic echo /camera/rightHipHand

    msg_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1); 
    // rostopic echo /iiwa/command/CartesianPose
    vibrate = n.advertise<std_msgs::UInt8>("/myo_raw/vibrate", 1); 

    ros::spin();

    return 0;
}
