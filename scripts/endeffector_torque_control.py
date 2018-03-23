#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from geometry_msgs.msg import WrenchStamped

'''
Node that subscribes to 'iiwa/state/CartesianWrench' and 'CModelRobotInput', and publishes to 'CModelRobotOutput'.
Given a certain force or torque, in combination with a gripper state, generates a command for the gripper to open or close. 
'''

state=2
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
threshold_torque_x = 1
threshold_force_x = 1
threshold_force_y = 320
prev_torque_x = 0
prev_force_x = 0 

def set_command(torque_diff, command):

    if torque_diff==6: #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 170
        command.rFR  = 25
        command.rPR = 0

    if torque_diff==7: #reset
        command.rACT = 0;

    if torque_diff==1: #open
        command.rPR = 0

    if torque_diff==2: #close
        command.rPR = 255

    
def force_control_sub():
    rospy.Subscriber("CModelRobotInput", inputMsg.CModel_robot_input, callback_state)  
    rospy.Subscriber("/iiwa/state/CartesianWrench", WrenchStamped, callback_wrench)       
    rospy.spin()


def callback_wrench(data):
    set_command(get_command_diff(data), command)  #Use get_command or get_command_diff
    pub.publish(command)

def callback_state(data):
    global state
    state=data.gOBJ
    rospy.loginfo("state = %s", state)

def get_command(data):

    torque_x = data.wrench.torque.x              
    force_x = data.wrench.force.x   
    force_y = data.wrench.force.y
  
    if abs(torque_x) > threshold_torque_x and state == 2 and command.rPR == 255:  #If the gripper is closed around an object and torque is applied -> open
        rospy.loginfo("vridmoment = %s", torque_x)
        return 1
    elif abs(force_x) > threshold_force_x and state == 3 and command.rPR == 0 :  #If the gripper is opened and force is applied in x-direction -> close
        rospy.loginfo("kraft i x-led = %s", force_x)
        return 2
    elif abs(force_y) > threshold_force_y and state == 3 and command.rPR == 255:  #If the gripper is closed and force is applied in y-direction -> open
        rospy.loginfo("kraft i y-led = %s", force_y)
        return 1


def get_command_diff(data):
    
    global prev_torque_x
    global prev_force_x
    torque_x = data.wrench.torque.x              
    force_x = data.wrench.force.x 
    torque_x_diff = abs(torque_x-prev_torque_x)
    force_x_diff = abs(force_x-prev_force_x)
    prev_torque_x = torque_x
    prev_force_x = force_x
    
    #if force_x_diff > threshold_force_x: rospy.loginfo("Diff force = %s", force_x_diff)
      
    if torque_x_diff > threshold_torque_x and state != 0 and command.rPR == 255:  #If the gripper is closed around an object and torque is applied -> open
        rospy.loginfo("(IF) Diff torque = %s", torque_x_diff)
        return 1
    elif force_x_diff > threshold_force_x and state != 0 and command.rPR == 0 :  #If the gripper is opened and force is applied in x-direction -> close
        rospy.loginfo("(IF) Diff force = %s", force_x_diff)
        return 2
    else:
        return 20
  

if __name__ == '__main__':
    rospy.init_node('force_control', anonymous=True)
    set_command(7, command)
    pub.publish(command)
    time.sleep(3)

    set_command(6, command)
    pub.publish(command)
    time.sleep(3)

    #set_command(2, command)
    #pub.publish(command)
    #time.sleep(3)

    rospy.loginfo("Go!")
    force_control_sub()
