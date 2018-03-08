#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from iiwa_msgs.msg import JointTorque

pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
torque = [0,0,0,0,0,0,0]  
prev_torque = [0,0,0,0,0,0,0]  
    
def robot_torque_sub():
    rospy.Subscriber("/iiwa/state/JointTorque", JointTorque, callback)      
    rospy.spin()

def callback(data):
    compare(data)

#compare torque to previous torque, if different return 1
def compare(data):
    prev_torque = list(torque)
    torque[0] = data.torque.a1
    torque[1] = data.torque.a2
    torque[2] = data.torque.a3
    torque[3] = data.torque.a4
    torque[4] = data.torque.a5
    torque[5] = data.torque.a6
    torque[6] = data.torque.a7
    diff = 0
    for x in range(7):
        diff = (torque[x]-prev_torque[x])**2
        if diff> 1:
            rospy.loginfo("diff on %s = %s", x, diff)

if __name__ == '__main__':
    rospy.init_node('torque_controller', anonymous=True)
    robot_torque_sub()
    
