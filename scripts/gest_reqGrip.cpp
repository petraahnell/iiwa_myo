#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"

ros::Publisher grip_pub;
ros::Subscriber gest_sub;

void reqGripper (const std_msgs::UInt8::ConstPtr& msg) {
	std_msgs::String s;
	if (msg->data == 1){		//make fist
		s.data = "o";		//send open command: "o"
		
	}
	else if (msg->data == 2){	//flex wrist to left
		s.data = "c";		//send close command: "c"
	}
	else {
		s.data = "No grip command";
	}
	grip_pub.publish(s);
	
}

int main (int argc, char **argv) {
	
	ros::init (argc, argv, "gest_to_req_grip");
	
	ros::NodeHandle n;
	grip_pub = n.advertise<std_msgs::String>("req_grip", 1000);
	gest_sub = n.subscribe("myo_gest", 1000, reqGripper);
	
	ros::spin();

	return 0;
}

