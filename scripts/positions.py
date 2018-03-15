#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float32
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity


def test():
	test = JointQuantity()

	test.a1 = 0.0
	test.a2 = 1.0
	test.a3 = 0.0
	test.a4 = -0.5
	test.a5 = 0.0
	test.a6 = 0.0
	test.a7 = 0.0

	return test

def handover():
	handover = JointQuantity()

	handover.a1 = 2.6245
	handover.a2 = -0.6626
	handover.a3 = 0.4108
	handover.a4 = 1.8504
	handover.a5 = -0.2260
	handover.a6 = 0.9733
	handover.a7 = 0.1937
	
	return handover

def home():
	home = JointQuantity()

	home.a1 = 0
	home.a2 = 0
	home.a3 = 0
	home.a4 = 0
	home.a5 = 0
	home.a6 = 0
	home.a7 = 0

	return home
