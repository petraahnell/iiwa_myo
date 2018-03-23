#!/usr/bin/env python
# Test to understand move group

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
import positions as p
from std_msgs.msg import UInt8

# make sure the needed messages are imported.

def move_group_iiwa(): 
    ## Setup stuff, i think...

    ## Initialize movait commander and rospy    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_iiwa', anonymous=True)
    
    ## Instansiate robot commander object
    robot = moveit_commander.RobotCommander()

    ## Planning interface, the world around the robot, is this needed??
    scene = moveit_commander.PlanningSceneinterface()

    ## Instanziate move group commander object. Interface to group of joints.
    group = moveit_commander.MoveGroupCommander("all joints") #This is probably wrong...

    ## This is done so RVIZ can vizualize.
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    ## sleep in order for RVIZ to start
    rospy.sleep(10)

	## This section should set a pose to go to.
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.a1 = 0
    pose_target.a2 = 0
    pose_target.a3 = 0
    pose_target.a4 = 0
    pose_target.a5 = 0
    pose_target.a6 = 0
    pose_target.a7 = 0
    
    ## I think this sets the pose....
    group.set_pose_target(pose_target)
    ## Now we plan
    plan1 = group.plan()

    ## Here we try to vizualize the plan 
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher-publish(display_trajectory)
    ## Sleep to let RVIZ vizualise
    rospy.sleep(5)
    
    # Uncomment below line when working with a real robot
    # group.go(wait=True)

    # Use execute instead if you would like the robot to follow
    # the plan that has already been computed
    # group.execute(plan1)


if __name__=='__main__':
    try:
        move_group_iiwa()

    except rospy.ROSInterruptException:
        pass




 


