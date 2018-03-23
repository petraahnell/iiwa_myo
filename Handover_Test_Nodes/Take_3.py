#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from geometry_msgs.msg import WrenchStamped
from iiwa_msgs.msg import JointVelocity
from iiwa_msgs.msg import JointPosition

'''
nod som ser nastan likadan ut som gripper_control men subsrcibar pa topic iiwa/state/CartesianWrench istallet
subscribes to the robots iiwa/state/CartesianWrench and publishes to the grippers CModelRobotOutput
opnnar med vridmoment kring i x
'''

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
threshold_x = 1.2   #vridmoment som behovs for att oppna grippern (z-riktning)


def gen_command(torque_diff, command):

    if torque_diff==6: #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 170
        command.rFR  = 25
        command.rPR = 0     #ny rad for att bestamma rPR varde innnan get(data) funktionen

    if torque_diff==7: #reset
        command.rACT = 0;

    if torque_diff==1: #open
        command.rPR = 0

    if torque_diff==2: #close
        command.rPR = 255

    
def force_control_sub():
    rospy.Subscriber("/iiwa/state/CartesianWrench", WrenchStamped, callback1)     
    rospy.spin()     
   

def callback1(data):
    gen_command(get(data), command)
    pub.publish(command)


def get(data):
    torque_x = data.wrench.torque.x    # is this really the correct one?
    
    if torque_x > threshold_x and command.rPR == 255:   #if the torque in x-direction is larger than threshold_x and the gripper is closed, return 1 to open gripper.
        rospy.loginfo("torque x = %s", torque_x)
        return 1
    
def gripper_init():
    """Ititializes the node and activates the gripper"""
    rospy.init_node('Take_3', anonymous=True)
    gen_command(7, command)
    pub.publish(command)
    gen_command(6, command)
    pub.publish(command)
    gen_command(2, command)
    pub.publish(command)

if __name__ == '__main__':
    gripper_init()
    rospy.loginfo("Go!")
    force_control_sub()
