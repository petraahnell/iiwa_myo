#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"

ros::Publisher robot_pub;       //Init robot run publisher
ros::Subscriber gest_sub;       //Init gesture subscriber
ros::Subscriber robot_sub;      //Init robot state subscriber
std_msgs::Bool run_msg;         //Init message to be published in robot_pub
bool run_goto;
bool robot_atPos;

void reqRunRobot (const std_msgs::UInt8::ConstPtr& msg) {
        if ((msg->data == 3) && (robot_atPos == false)){		//flex wrist to right and robot not at pick position
                run_goto = true;
	}
        run_msg.data = run_goto;
        robot_pub.publish(run_msg);                                     //Publish if the robot should run or not
}

void quitReqRun (const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data == true) {                                        //robot is at pick position
		robot_atPos = true;
                run_goto = false;

                run_msg.data = run_goto;
                robot_pub.publish(run_msg);                             //update the run_goto variable
	}
}

void setUp(){
    run_goto = false;
    robot_atPos = false;
}

int main (int argc, char **argv) {
	
        setUp();        //Run setup of variables.
	ros::init (argc, argv, "gest_to_req_grip");
	
	ros::NodeHandle n;
	
	robot_pub = n.advertise<std_msgs::Bool>("goto_handover_pos", 1000);
	gest_sub = n.subscribe("myo_gest", 1000, reqRunRobot);
	robot_sub = n.subscribe("at_handover_pos", 100, quitReqRun);
	
        ros::spin();        //Keep program running

	return 0;
}

