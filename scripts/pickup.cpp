#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointQuantity.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pickup");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  //sleep(20.0);

  std::string movegroup_name = "manipulator";
  moveit::planning_interface::MoveGroup group(movegroup_name);

  group.setPlanningTime(0.5);
  group.setPlannerId(movegroup_name+"[RRTConnectConfigDefault]");
  group.setEndEffectorLink("iiwa_link_ee");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  std::vector<double> joint_target;
  joint_target.push_back(0);
  joint_target.push_back(0.809746382194823);
  joint_target.push_back(0);
  joint_target.push_back(-1.355374742864191);
  joint_target.push_back(0);
  joint_target.push_back(0.933131840114064);
  joint_target.push_back(0);
  group.setStartStateToCurrentState();
  group.setJointValueTarget(joint_target);  
/*
  iiwa_msgs::JointQuantity target;
  target.a1 = 0;
  target.a2 = 0.809746382194823;
  target.a3 = 0;
  target.a4 = -1.355374742864191;
  target.a5 = 0;
  target.a6 = 0.933131840114064;
  target.a7 = 0;
  group.setStartStateToCurrentState();
  group.setJointValueTarget(target);

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.626522;
  target_pose1.position.y = 5.98164e-05;
  target_pose1.position.z = 0.266002;
  target_pose1.orientation.x = -6.99428e-05;
  target_pose1.orientation.y = 0.999958;
  target_pose1.orientation.z = -3.07521e-05;
  target_pose1.orientation.w = -0.00916401;
  group.setStartStateToCurrentState();
  group.setJointValueTarget(target_pose1);
*/
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);
  
  group.move();

  geometry_msgs::Pose target_pose1;
  target_pose1 = group.getCurrentPose().pose;
  target_pose1.position.z -= 0.1;
  group.setStartStateToCurrentState();
  group.setPoseTarget(target_pose1);

  group.setMaxVelocityScalingFactor(0.01);
  group.move();
 
// END_TUTORIAL

  ros::shutdown();  
  return 0;
}

