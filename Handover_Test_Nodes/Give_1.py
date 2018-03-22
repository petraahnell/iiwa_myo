#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from geometry_msgs.msg import Vector3

command = outputMsg.CModel_robot_output()
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)

def gen_command(gest, command):  
    """Generates a gripper command given a certain gesture"""
    if gest==6: #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 105
        command.rFR  = 25

    if gest==7: #reset
        command.rACT = 0

    if gest==4: #open open palm
        command.rPR = 0

    if gest== 20: #close
        command.rPR = 255 

  
def gripper_control_sub():
    """Subscribe to gestures from myo"""
    rospy.Subscriber("/myo_raw/myo_ori_deg", Vector3, callback)
    rospy.spin()

def gripper_init():
    """Ititializes the node and activates the gripper"""
    rospy.init_node('gripper_control', anonymous=True)
    gen_command(4, command)
    pub.publish(command)

def callback(data):
    """When recieving a gesture, print it in the terminal and publish a command to the gripper."""
    roll = data.z
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", roll)

    if roll < -45:
        gen_command(20, command)
        pub.publish(command)

if __name__ == '__main__':
    gripper_init()
    gripper_control_sub()
