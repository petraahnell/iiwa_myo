#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <robotiq_c_model_control/CModel_robot_output.h>
#include <ros_myo/MyoPose.h> 
#include <geometry_msgs/WrenchStamped.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

ros::Publisher gripper_pub;
ros::Subscriber gest_sub, robot_sub;

robotiq_c_model_control::CModel_robot_output gripper;

int force_give, force_take, torque_take;
int force_give_lim = -10;
int force_take_lim = 5;
int torque_take_lim = 1.2;

bool atPick, atDrop, pickup, drop, gotoPick, atHandover, gotoDrop, gripperClosed = false;
bool gotoHandover = true;
bool init = true;

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

void sequence(){
    std::string movegroup_name = "manipulator";
    moveit::planning_interface::MoveGroup group(movegroup_name);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    group.setPlanningTime(0.5);
    group.setPlannerId(movegroup_name+"[RRTConnectConfigDefault]");
    group.setEndEffectorLink("iiwa_link_ee");

    if (gotoPick){
        atHandover = false;        

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

	    group.move();
	    //sleep(3.0);

        gotoPick = false;
        atPick = true;
    }

    if (pickup && atPick && !gripperClosed){   //if grip registered
	    geometry_msgs::Pose target;
	    target = group.getCurrentPose().pose;
	    target.position.z -= 0.1;
	    group.setPoseTarget(target);
	    group.setMaxVelocityScalingFactor(0.01);
	    group.move();
	    //sleep(5.0);
		
	    gripper.rPR = 255;     //close gripper   
	    gripper_pub.publish(gripper);
	    gripperClosed = true;
        sleep(1.5);
	
	    target = group.getCurrentPose().pose;
	    target.position.z += 0.1;
        //target.position.y = 0.0;
	    group.setPoseTarget(target);
	    group.move();
        group.setMaxVelocityScalingFactor(1.0);
	    //sleep(5.0);

        pickup = false;
    }

    if (gotoHandover){
        atPick = false;
        atDrop = false;

	    std::vector<double> handover_pos;
	    handover_pos.push_back(0.0);
	    handover_pos.push_back(0.5910);
	    handover_pos.push_back(0);
	    handover_pos.push_back(-1.8215);
	    handover_pos.push_back(0.0);
	    handover_pos.push_back(-0.8326);
	    handover_pos.push_back(0.3733);
	    group.setStartStateToCurrentState();
	    group.setJointValueTarget(handover_pos); 

	    group.move();
	    sleep(0.5);

        gotoHandover = false;
        atHandover = true;
    }

    if (gotoDrop){
        atHandover = false;

	    std::vector<double> drop_pos;
	    drop_pos.push_back(0.4270);
	    drop_pos.push_back(0.2394);
	    drop_pos.push_back(0);
	    drop_pos.push_back(-1.5441);
	    drop_pos.push_back(0.0);
	    drop_pos.push_back(1.3569);
	    drop_pos.push_back(1.4092);
	    group.setStartStateToCurrentState();
	    group.setJointValueTarget(drop_pos); 

	    group.move();
	    //sleep(5.0);

        gotoDrop = false;
        atDrop = true;
    }

    if (drop && atDrop && gripperClosed){
        gripper.rPR = 0;     //open gripper   
	    gripper_pub.publish(gripper);
	    gripperClosed = false;
        sleep(1.0);
        
        drop = false;
    }

    //ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    //ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    ROS_INFO("END OF SEQUENCE");
}

void gest(const ros_myo::MyoPose::ConstPtr& msg){
    if (msg->pose == 3){
        if (atHandover){ gotoPick = true; }
        if (atDrop){ gotoHandover = true; }
    }

    else if (msg->pose == 2 && atPick){
        pickup = true;
    }

    else if (msg->pose == 5 && atDrop){
        drop = true;
    }

    else if (msg->pose == 4){
        if (atPick) { gotoHandover = true; }
        if (atHandover) { gotoDrop = true; }
    }

    ROS_INFO("Gest is %d", msg->pose);
    sequence();
}

void torque_force(const geometry_msgs::WrenchStamped msg) {
    force_give = msg.wrench.force.y;
    torque_take = msg.wrench.torque.x;
    force_take = msg.wrench.force.z;

    if(atHandover){
        if(gripperClosed){
            if(force_take > force_take_lim || torque_take > torque_take_lim){
                gripper.rPR = 0;     //open gripper   
	            gripper_pub.publish(gripper);
	            gripperClosed = false;
                sleep(1);
            }
        } else { 
            if(force_give < force_give_lim){
                gripper.rPR = 255;     //close gripper   
	            gripper_pub.publish(gripper);
	            gripperClosed = true;
                sleep(1);
            }
        }
    }
    //ROS_INFO("Force torque loop");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pickup");
    ros::NodeHandle n;  
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /* This sleep is ONLY to allow Rviz to come up*/
    //sleep(20.0);  

    ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    gest_sub = n.subscribe("/myo_raw/myo_gest", 1, gest);
    robot_sub = n.subscribe("/iiwa/state/CartesianWrench", 1, torque_force);

    gripper_pub = n.advertise<robotiq_c_model_control::CModel_robot_output>("/CModelRobotOutput", 1);

    if (init) {
        initGripper();
        //sequence();
        init = false;
    }

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


    ros::spin();  
    return 0;
}

