#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <robotiq_c_model_control/CModel_robot_output.h>
#include "ros_myo/MyoPose.h" 

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

ros::Publisher gripper_pub;
ros::Subscriber gest_sub;

robotiq_c_model_control::CModel_robot_output gripper;

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

void sequence(const ros_myo::MyoPose::ConstPtr& msg){
	if (msg->pose == 3){
		std::vector<double> pickup_pos;
		pickup_pos.push_back(-0.5232);
		pickup_pos.push_back(0.3233);
		pickup_pos.push_back(0);
		pickup_pos.push_back(-1.7054);
		pickup_pos.push_back(-0.0001);
		pickup_pos.push_back(1.1114);
		pickup_pos.push_back(1.4091);
		group.setStartStateToCurrentState();
		group.setJointValueTarget(pickup_pos); 

		moveit::planning_interface::MoveGroup::Plan my_plan;
		bool success = group.plan(my_plan);

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 

		group.move();
		sleep(5.0);
	}

	if ((msg->pose == 2) && !gripperClosed){   //if gripp registered
		geometry_msgs::Pose target;
		target = group.getCurrentPose().pose;
		target.position.z -= 0.1;
		group.setPoseTarget(target);
		group.setMaxVelocityScalingFactor(0.01);
		group.move();
		sleep(5.0);
			
		gripper.rPR = 255;     //close gripper   
		gripper_pub.publish(gripper);
		gripperClosed = true;
		
		target = group.getCurrentPose().pose;
		target.position.z += 0.1;
		group.setPoseTarget(target);
		//group.setMaxVelocityScalingFactor(0.01);
		group.move();
		sleep(5.0);
	}

	if (msg->pose == 5){
		std::vector<double> handover_pos;
		pickup_pos.push_back(0.0);
		pickup_pos.push_back(1.2394);
		pickup_pos.push_back(0);
		pickup_pos.push_back(-0.8913);
		pickup_pos.push_back(0.0);
		pickup_pos.push_back(-0.5569);
		pickup_pos.push_back(1.9349);
		group.setStartStateToCurrentState();
		group.setJointValueTarget(handover_pos); 

		moveit::planning_interface::MoveGroup::Plan my_plan;
		bool success = group.plan(my_plan);

		ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 

		group.move();
		sleep(5.0);
	}
}


void handover()
{
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pickup");
  ros::NodeHandle n;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up*/
  //sleep(20.0);

  std::string movegroup_name = "manipulator";
  moveit::planning_interface::MoveGroup group(movegroup_name);

  group.setPlanningTime(5.0);
  group.setPlannerId(movegroup_name+"[RRTConnectConfigDefault]");
  group.setEndEffectorLink("iiwa_link_ee");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

/*
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.428;
  target_pose1.position.y = -0.246;
  target_pose1.position.z = 0.356;
  target_pose1.orientation.x = 0.832;
  target_pose1.orientation.y = 0.555;
  target_pose1.orientation.z = -0.0036;
  target_pose1.orientation.w = -0.0055;
  group.setStartStateToCurrentState();
  group.setJointValueTarget(target_pose1);
*/


  ros::shutdown();  
  return 0;
}

