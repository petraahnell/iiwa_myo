#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_listener.h"
#include "ros_myo/MyoPose.h" 
#include "math.h"

ros::Publisher msg_pub;
ros::Subscriber pos_sub;
ros::Subscriber gest_sub;
ros::Subscriber ori_sub;

geometry_msgs::PoseStamped my_pos;
geometry_msgs::Pose pose;
geometry_msgs::Point p;
geometry_msgs::Quaternion q;

bool init = true;
bool gripRegistered = false;     //init to 'false' if to use with myo-band
bool posFinished = false;
bool qFinished = false;
int posCounter = 1;
int qCounter = 1;
const int NRCALIB = 25;

void posteMsg(){
    pose.position = p;
    pose.orientation = q;
    my_pos.pose = pose;
    
    msg_pub.publish(my_pos);
    ROS_INFO("Pose calibration finished!");
}


void msgControl(const ros_myo::MyoPose::ConstPtr& msg){  

  if(msg->pose == 2){       //if grip
    gripRegistered = true;
    ROS_INFO("Pose calibration started");
  }
}

void calibratePos(const geometry_msgs::Point::ConstPtr& msg){  
  if(gripRegistered && (posCounter < NRCALIB)){
    p.x = (p.x + msg->x) / 2;     //compute average
    p.y = (p.y + msg->y) / 2;
    p.z = (p.z + msg->z) / 2;
    
    posCounter++;
  }
  else if (posCounter >= NRCALIB){
    posFinished = true;
  }
}

void calibrateQ(const sensor_msgs::Imu::ConstPtr& msg){

    if(gripRegistered && (qCounter < NRCALIB)){
        q.x = (q.x + msg->orientation.x) / 2; 
        q.y = (q.y + msg->orientation.y) / 2; 
        q.z = (q.z + msg->orientation.z) / 2; 
        q.w = (q.w + msg->orientation.w) / 2;
        
        qCounter++;
    } 
    else if (qCounter >= NRCALIB){
        qFinished = true; 
    }
    if (posFinished && qFinished){
        posteMsg();
        posFinished, qFinished, gripRegistered = false;
        ros::shutdown();
    }
}

void initialize(){
    p.x, p.y, p.z = 0.0;    // init value
    q.x, q.y, q.z, q.w = 0.0;
    init = false;
}



int main(int argc, char **argv){

  ros::init(argc, argv, "calibration");
  ros::NodeHandle n;
    
  if (init){
    initialize();
  }

  pos_sub = n.subscribe("/camera/rightHipHand", 100, calibratePos);
  gest_sub = n.subscribe("/myo_raw/myo_gest", 1, msgControl);
  ori_sub = n.subscribe("/myo_raw/myo_imu", 100, calibrateQ);
  msg_pub = n.advertise<geometry_msgs::PoseStamped>("/arm_imitation/Calibration", 1);

  ros::spin();

  return 0;
}
