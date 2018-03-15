#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <vector>
#include <std_msgs/String.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/SmartServoMode.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>


