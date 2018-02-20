#!/usr/bin/env python
#import roslib; roslib.load_manifest('robotiq_c_model_control')
#from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
from std_msgs.msg import UInt8 

    
def gripper_control_sub():

    rospy.init_node('gripper_control', anonymous=True)
    rospy.Subscriber("myo_gest", UInt8, callback)
    rospy.spin()

pub = rospy.Publisher('helloworld', UInt8)
#pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish(7)


if __name__ == '__main__':
    gripper_control_sub()
