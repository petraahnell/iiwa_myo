#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
import positions as p

def talker():
    posPub = rospy.Publisher('iiwa/command/JointPosition', JointPosition, queue_size=10)
    rospy.init_node('command_JointPosition', anonymous =False)

    rate = rospy.Rate(0.5) 
    x = 0
    rospy.loginfo("Talker started")
    while not rospy.is_shutdown():
 
        pos = JointPosition()
        command = raw_input('Enter a valid position: ') #Positions can be found in the file positions.py
        if command == 'home':
            pos.position = p.home()
        elif command == 'handover':
            pos.position = p.handover()
        elif command == 'test':
            pos.position = p.test()
        else:
            rospy.loginfo('Not a valid position, quitting.')
            rospy.signal_shutdown('A valid position was not entered.')
            break

        rospy.loginfo(str(x))

        x = x + 1

        pos.header= Header()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = ''
        pos.header.seq=x
        rospy.loginfo(pos.position)
        rospy.loginfo("Position will be transmitted")
        posPub.publish(pos)
        rospy.loginfo("Position has been transmitted")
		
        rate.sleep()


if __name__ == '__main__':
  try:
      talker()
  #velocity_talker()
  except rospy.ROSInterruptException:
      pass
