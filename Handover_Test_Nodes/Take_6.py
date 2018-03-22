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
opnnar med kraft i z-riktning
'''

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
threshold_z = 800   #kraft som behovs for att oppna grippern (z-riktning)


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
    force_z = data.wrench.force.z    
    
  
    if abs(force_z) > threshold_z and command.rPR == 255:   #if the force in z-direction is larger than threshold_z and the gripper is closed, return 1 to open gripper.
        rospy.loginfo("force z = %s", force_z)
        return 1
    

if __name__ == '__main__':
    rospy.init_node('force_control', anonymous=True) 
    rospy.loginfo("Go!")
    force_control_sub()
