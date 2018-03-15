#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
import positions as p
from std_msgs.msg import UInt8


def myo_control_sub():
    """Subscribe to gestures from myo"""
    rospy.Subscriber("myo_gest", UInt8, callback)
    rospy.init_node('myo_JointPosition', anonymous =False)

    rospy.spin()

def callback(data):
    """When recieving a gesture, print it in the terminal and publish a command to the gripper."""
    command = data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", command)


    posPub = rospy.Publisher('iiwa/command/JointPosition', JointPosition, queue_size=10)
    

    #rate = rospy.Rate(0.5) # 10hz
    x = 0
    rospy.loginfo("Talker started")
    #while not rospy.is_shutdown():
 
    pos = JointPosition()
    #command = raw_input('Enter a valid position: ') #Positions can be found in the file positions.py
    if command == 3:
        pos.position = p.home()
        pos.header= Header()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = ''
        pos.header.seq=x
        rospy.loginfo(pos.position)
        rospy.loginfo("Position will be transmitted")
        posPub.publish(pos)
        rospy.loginfo("Position has been transmitted")
    elif command == 2:
        pos.position = p.handover()
        pos.header= Header()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = ''
        pos.header.seq=x
        rospy.loginfo(pos.position)
        rospy.loginfo("Position will be transmitted")
        posPub.publish(pos)
        rospy.loginfo("Position has been transmitted")
    elif command == 9:
        pos.position = p.test()
        pos.header= Header()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = ''
        pos.header.seq=x
        rospy.loginfo(pos.position)
        rospy.loginfo("Position will be transmitted")
        posPub.publish(pos)
        rospy.loginfo("Position has been transmitted")
  

    rospy.loginfo(str(x))

    x = x + 1

		
    #rate.sleep()


if __name__ == '__main__':
     myo_control_sub()
     try:
         pass
         #velocity_talker()
     except rospy.ROSInterruptException:
         pass
