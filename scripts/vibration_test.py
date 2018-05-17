#!/usr/bin/env python
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
import rospy
import time
from std_msgs.msg import UInt8 
from ros_myo.msg import MyoPose

pub = rospy.Publisher('/myo_raw/vibrate', UInt8, queue_size=10)

def gest_sub():
    """Subscribe to gest from myo"""
    rospy.Subscriber("/myo_raw/myo_gest", MyoPose, callback)
    rospy.spin()

def callback(data):
    gest = data.pose
    if gest == 2:
        pub.publish(2)
        time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('vibration_test', anonymous=True)
    gest_sub()
