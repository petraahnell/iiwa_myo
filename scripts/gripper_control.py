#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
from std_msgs.msg import UInt8 


def genCommand(data, command):  
    #command = outputMsg.CModel_robot_output();
    if data.data==4:    #activate
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 105
        command.rFR  = 25

    if data.data==1:   #oppnar
        command.rPR = 0

  
    
def gripper_control_sub():
    rospy.init_node('gripper_control', anonymous=True)
    rospy.Subscriber("myo_gest", UInt8, callback)
    rospy.spin()

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)

command = outputMsg.CModel_robot_output()
gest = 0

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    gest = data.data
    genCommand(data, command)
    pub.publish(command)

if __name__ == '__main__':
    gripper_control_sub()
