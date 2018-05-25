#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "ros_myo/MyoPose.h" 
#include "math.h"

ros::Publisher msg_pub;
ros::Subscriber pos_sub, gest_sub, calib_sub, arm_ori_sub, robot_ori_sub;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point p;
geometry_msgs::Quaternion q;

tf::Quaternion q0a, q0r, qRelative, Qa, Qr;

bool calibrated, gripRegistered, armOriUpdated, firstGrip, robotOriUpdated, robotInit = false;

float REST_POS_X;    //Distance from hip to hand in z when arm is in "rest position"
float REST_POS_Z;    //Distance from hip to hand in x when arm is in "rest position"
float QXold, QYold, QZold, QWold, QXnew, QYnew, QZnew, QWnew, QXd, QYd, QZd, QWd;     //Quaternions for users arm (old, new, difference)
float rQXold, rQYold, rQZold, rQWold, rQXnew, rQYnew, rQZnew, rQWnew, rQXd, rQYd, rQZd, rQWd;     // Quaternions for robot (old, new, difference)

//const float MAX_POS_Y = 0.61;       //Distance from hip to hand in y-direction when arm is straight out to the side
const float MIN_VALUE_Z = 0.2;    //Minimun value allowed in z-direction. Safety to not hit table etc.
const float ROBOT_GOAL_POS_X = 0.6;    //The position we want the robot to have in x-direction when in "rest position"
const float ROBOT_GOAL_POS_Z = 0.5;    //The position we want the robot to have in z-direction when in "rest position"
//const float ROBOT_GOAL_POS_Y = 1.2;
#define PI 3.14159265

void calibration(const geometry_msgs::PoseStamped::ConstPtr& msg){
    
    REST_POS_X = msg->pose.position.x;
    REST_POS_Z = msg->pose.position.z;
    QXold = msg->pose.orientation.x;
    QYold = msg->pose.orientation.y;
    QZold = msg->pose.orientation.z;
    QWold = msg->pose.orientation.w;
    calibrated = true;
    ROS_INFO("Arm imitation ready");

    q0a = tf::Quaternion(QXold, QYold, QZold, QWold);

    //Init
    QXd, QYd, QZd, QWd = 0;
    //Test
    rQXd, rQYd, rQZd, rQWd = 0;
    rQXold = 0;
    rQYold = 1.711;
    rQZold = 0;
    rQWold = 1.703;
    q0r = tf::Quaternion(rQXold, rQYold, rQZold, rQWold);
    
    qRelative = q0r*q0a.inverse();
}

void msgControl (const ros_myo::MyoPose::ConstPtr& msg){
  
    if ((msg->pose == 3) && !gripRegistered){
        gripRegistered = true;
        firstGrip = true;
    }
    else if ((msg->pose == 3) && firstGrip){
        firstGrip = false;
    }
    else{
        gripRegistered = false;
    }
}

void armOriControl (const sensor_msgs::Imu::ConstPtr& msg){
    if (calibrated && gripRegistered) {  // && robotOriUpdated
        if (firstGrip){
            QXold = msg->orientation.x;
            QYold = msg->orientation.y;
            QZold = msg->orientation.z;
            QWold = msg->orientation.w;
            firstGrip = false;
        }        
        // Update quaternions
        QXnew = msg->orientation.x;
        QYnew = msg->orientation.y;
        QZnew = msg->orientation.z;
        QWnew = msg->orientation.w;

        Qa = tf::Quaternion(QXnew, QYnew, QZnew, QWnew);
        
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

        //Test
        float delay = 0.5;
        rQXnew = QXnew - delay * QXd;
        rQYnew = QYnew - delay * QYd;
        rQZnew = QZnew - delay * QZd;
        rQWnew = QWnew - delay * QWd;

        rQXd = rQXnew - rQXold;   
        rQYd = rQYnew - rQYold;   
        rQZd = rQZnew - rQZold;   
        rQWd = rQWnew - rQWold;   

        rQXold = rQXnew;
        rQYold = rQYnew;
        rQZold = rQZnew;
        rQWold = rQWnew;
    }
}

void robotOriControl (const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (!robotInit){
        rQXold = msg->pose.orientation.x;
        rQYold = msg->pose.orientation.y;
        rQZold = msg->pose.orientation.z;
        rQWold = msg->pose.orientation.w;
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

    robotOriUpdated = true;
}

void makeMsg (const geometry_msgs::Point::ConstPtr& msg){
  
    if (calibrated && gripRegistered && armOriUpdated){     //OBS: Kom ihåg robotOriUpdated !!

        //x-position
        float x_pos = msg->x;
        float Ax = x_pos / REST_POS_X;
        float Bx = ROBOT_GOAL_POS_X - REST_POS_X;

        p.x = ROBOT_GOAL_POS_X*fmin(Ax, sqrt(Ax));
        if (p.x < 0.6){
            p.x = 0.6;
        }
        else if (p.x > 0.8){
            p.x = 0.8;
        }
        
        //y-position
        float y_pos = msg->y;
        p.y = msg->y;             //Set pos
      
        if (p.y > 0.8){
            p.y = 0.8;
        } 
        //z-position
        float Az = msg->z / REST_POS_Z;
        float z_pos = ROBOT_GOAL_POS_Z*fmin(Az, sqrt(Az));      //Set pos
        
        if (z_pos < MIN_VALUE_Z){      //Do not go to low bc table.
            p.z = MIN_VALUE_Z;
        }
        else if (z_pos > sqrt(1.1*1.1 - p.x*p.x - y_pos*y_pos)){
            p.z = sqrt(1.1*1.1 - p.x*p.x - y_pos*y_pos);
        }
        else{
            p.z = z_pos;  
        }

        //Quaternion
        tf::Quaternion Qfinal = qRelative.operator*=(Qa);
        q.x = (float) Qfinal.getAxis().x(); //rQXnew + QXd; //  1 + QXd;
        q.y = (float) Qfinal.getAxis().y(); //rQYnew + QYd; // 0.711 + QYd;
        q.z = (float) Qfinal.getAxis().z(); //rQZnew + QZd; //   1 + QZd;
        q.w = Qfinal.getW(); //rQWnew + QWd; //  0.703 + QWd; 
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

    calib_sub = n.subscribe("/arm_imitation/Calibration", 1, calibration);      // rostopic echo /arm_imitation/Calibration
    gest_sub = n.subscribe("/myo_raw/myo_gest", 1, msgControl);      // rostopic echo /myo_raw/myo_gest
    arm_ori_sub = n.subscribe("/myo_raw/myo_imu", 1, armOriControl);      // rostopic echo /myo_raw/myo_imu
    robot_ori_sub = n.subscribe("/iiwa/state/CartesianPose", 1, robotOriControl);  // rostopic echo /iiwa/state/CartesianPose
    pos_sub = n.subscribe("/camera/rightHipHand", 1, makeMsg);      // rostopic echo /camera/rightHipHand

    msg_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 1); // rostopic echo /iiwa/command/CartesianPose

    ros::spin();

    return 0;
}
