#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"

 ros::Publisher pos_pub;
 ros::Subscriber sub;

 void myoGest(const std_msgs::UInt8::ConstPtr& msg) {
		iiwa_msgs::JointPosition my_pos; 
		iiwa_msgs::JointQuantity my_q;
	if (msg->data == 0){
		my_q.a1 = 0.0;
		my_q.a2 = 1.0;
		my_q.a3 = 0.0;
		my_q.a4 = 0.0;
		my_q.a5 = 0.0;
		my_q.a6 = 0.0;
		my_q.a7 = 0.0;
		my_pos.position = my_q;
		ROS_INFO("I heard: [%d], so you are resting", msg->data);
	}
	else if (msg->data == 1){
		my_q.a1 = 0.5;
		my_q.a2 = 0.5;
		my_q.a3 = 0.5;
		my_q.a4 = 0.5;
		my_q.a5 = 0.5;
		my_q.a6 = 0.5;
		my_q.a7 = 0.5;
		my_pos.position = my_q;
		ROS_INFO("I heard: [%d], so you are gripping", msg->data);
	}
	pos_pub.publish(my_pos);
   //ROS_INFO("I heard: [%d]", msg->data);
 }

  
 int main(int argc, char **argv) {
 
   ros::init(argc, argv, "gest_to_joint_positions");
      
   ros::NodeHandle n;
   sub = n.subscribe("myo_gest", 1000, myoGest);
   pos_pub = n.advertise<iiwa_msgs::JointPosition>("joint_positions", 1000);

   ros::spin();
  
   return 0;
 }
