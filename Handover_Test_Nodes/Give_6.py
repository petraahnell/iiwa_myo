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
stanga med kraft i x-riktning
In the handoverposition the forces are (-5.8,-16.7, -0.05) and the torque is (0.90, -0.62, -0.05). Use force y and torque x.
'''

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
threshold_z = 5   #kraft som behovs for att stanga grippern (x-riktning,)
threshold_y = -20 

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
    force_y = data.wrench.force.y    
  
    if force_y < threshold_y and command.rPR == 0:   #if the force in y-direction is larger than threshold_y and the gripper is open, return 2 to close gripper.
        rospy.loginfo("force y = %s", force_y)
        return 2

    elif force_z > threshold_z and command.rPR == 255:   
        rospy.loginfo("force z = %s", force_z)
        return 1
    

if __name__ == '__main__':
    rospy.init_node('Give_6', anonymous=True) 
    gen_command(7, command)
    pub.publish(command)
    gen_command(6, command)
    pub.publish(command)

    rospy.loginfo("Go!")
    force_control_sub()
