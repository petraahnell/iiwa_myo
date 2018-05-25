#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"
#include "tf/transform_listener.h"
#include "ros_myo/MyoPose.h" 
#include "math.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"

ros::Publisher msg_pub;
ros::Subscriber gest_sub;

iiwa_msgs::JointPosition my_pos;
iiwa_msgs::JointQuantity jpos;

void msgControl (const ros_myo::MyoPose::ConstPtr& msg){
  
    if ((msg->pose == 3)){
        jpos.a1 = 0.0104654040188;
        jpos.a2 = 0.293048143387;
        jpos.a3 = -0.0143755571917;
        jpos.a4 = -1.86739754677;
        jpos.a5 = -0.0157163050026;
        jpos.a6 = -0.585920929909;
        jpos.a7 = 1.92783629894;


        //make msg
        my_pos.position = jpos;
        my_pos.header.stamp = ros::Time::now();
        my_pos.header.frame_id = "init_start_pose";
        msg_pub.publish(my_pos);
        ros::shutdown();
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "init_start_pose");
    ros::NodeHandle n;

    gest_sub = n.subscribe("/myo_raw/myo_gest", 1, msgControl);      
    // rostopic echo /myo_raw/myo_gest

    msg_pub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1); 
    // rostopic echo /iiwa/command/CartesianPose

    ros::spin();

    return 0;
}
