#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from geometry_msgs.msg import Vector3
import test_variables as v

command = outputMsg.CModel_robot_output()
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)

def gen_command(value, command):  
    """Generates a gripper command given a certain value"""
    if value==6: #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 105
        command.rFR  = 25

    if value==7: #reset
        command.rACT = 0

    if value==4: #open 
        command.rPR = 0

    if value==2: #close
        command.rPR = 255 

  
def gripper_control_sub():
    """Subscribe to orientation from myo"""
    rospy.Subscriber("/myo_raw/myo_ori_deg", Vector3, callback, queue_size=1, buff_size=65536)
    rospy.spin()

def gripper_init():
    """Ititializes the node and activates the gripper"""
    rospy.init_node('take_pitch_roll', anonymous=True)
    gen_command(7, command)
    pub.publish(command)
    gen_command(6, command)
    pub.publish(command)
    gen_command(2, command)
    pub.publish(command)

def callback(data):

    pitch = data.y
    roll = data.z
    rospy.loginfo("Pitch: %s, Roll: %s", pitch, roll)

    if pitch < v.pitch_lim:
        if roll < v.roll_lim:
            gen_command(4, command)
            pub.publish(command)
            value = int(raw_input("Write 2 for close: "))
            gen_command(value, command)
            pub.publish(command)

if __name__ == '__main__':
    gripper_init()
    gripper_control_sub()
