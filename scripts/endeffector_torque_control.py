#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from geometry_msgs.msg import WrenchStamped

'''
nod som ser nastan likadan ut som gripper_control men subsrcibar pa topic iiwa/state/CartesianWrench istallet
#subscribes to the robots iiwa/state/CartesianWrench and publishes to the grippers CModelRobotOutput
#oppnar med torque, stanger med kraft i x riktning
'''


pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
threshold_a = 5   #vridmoment som behovs for att oppna grippern
threshold_b = 15   #kraft i x-riktning som behovs for att stanga grippern


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
    rospy.Subscriber("/iiwa/state/CartesianWrench", WrenchStamped, callback)    
    rospy.spin()

def callback(data):
    gen_command(get(data), command)
    pub.publish(command)


def get(data):
    a = data.wrench.torque.x              #vilket vridmoment ger utlag vid a?       
    b = data.wrench.force.x              #kraft i x-riktning
  
    if abs(a) > threshold_a and command.rPR == 255:   #if the torque in direction is larger than threshold_a and the gripper is closed, return 1 to open gripper.
        rospy.loginfo("a = %s", a)
        return 1
    elif abs(b) > threshold_b and command.rPR == 0 :   #if the force in x-direction is larger than threshold_b and the gripper is open, return 2 to open gripper.
        rospy.loginfo("b = %s", b)
        return 2
    

if __name__ == '__main__':
    rospy.init_node('force_control', anonymous=True)
    gen_command(7, command)
    pub.publish(command)
    time.sleep(3)

    gen_command(6, command)
    pub.publish(command)
    time.sleep(3)

    #gen_command(2, command)
    #pub.publish(command)
    #time.sleep(3)

    rospy.loginfo("Go!")
    force_control_sub()
