#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from iiwa_msgs.msg import JointTorque

#nod som ser nastan likadan ut som gripper_control men subsrcibar pa topic JointTorque istallet
#subscribes to the robots JointTorque and publishes to the grippers CModelRobotOutput

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
torque_4 = 0
prev_torque_4 = 0
threshold = 0.05
first = True

def gen_command(torque_diff, command):

    if torque_diff==6: #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 170
        command.rFR  = 25

    if torque_diff==7: #reset
        command.rACT = 0;

    if torque_diff==1: #open
        command.rPR = 0

    if torque_diff==2: #close
        command.rPR = 255

    
def torque_controller_sub():
    rospy.Subscriber("/iiwa/state/JointTorque", JointTorque, callback)
    rospy.spin()

def callback(data):
    gen_command(compare(data), command)
    pub.publish(command)

#compare torque to previous torque, if different return 1
def compare(data):
    global first
    global torque_4
    prev_torque_4 = torque_4
    torque_4 = data.torque.a4
    a = 0
    for x in range(7):
        if first:
            a=0
        else:
            a = torque_4-prev_torque_4
    first=False
    if a>threshold:
        rospy.loginfo("a = %s", a)
        return 1

if __name__ == '__main__':
    rospy.init_node('torque_controller', anonymous=True)
    gen_command(7, command)
    pub.publish(command)
    time.sleep(3)

    gen_command(6, command)
    pub.publish(command)
    time.sleep(3)

    gen_command(2, command)
    pub.publish(command)
    time.sleep(3)

    rospy.loginfo("Go!")
    torque_controller_sub()
