#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 


def genCommand(gest, command):  
    #command = outputMsg.CModel_robot_output();
    if gest==6:    #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 105
        command.rFR  = 25

    if gest==1:   #open/close
        command.rPR = 255

    if gest==2:    #close
        command.rPR = 255 
    
def gripper_control_sub():
    #rospy.init_node('gripper_control', anonymous=True)
    rospy.Subscriber("myo_gest", UInt8, callback)
    rospy.spin()

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
command = outputMsg.CModel_robot_output()
gest = 0

def callback(data):
    gest = data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", gest)
    genCommand(gest, command)
    pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('gripper_control', anonymous=True)
    genCommand(6, command)
    pub.publish(command)
    time.sleep(3)
    genCommand(2, command)
    pub.publish(command)
    gripper_control_sub()
