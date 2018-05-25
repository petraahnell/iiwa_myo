#!/usr/bin/env python
import rospy
from scipy.signal import butter, lfilter
from scipy.signal import freqs
from geometry_msgs.msg import Point

global r_posXa, r_posYa, r_posZa
r_posXa = [0]
r_posYa = [0]
r_posZa = [0]

global l_posXa, l_posYa, l_posZa
l_posXa = [0]
l_posYa = [0]
l_posZa = [0]

global order, fs, cutoff
# Filter requirements.
order = 3
fs = 30.0       # sample rate, Hz
cutoff = 0.8    # desired cutoff frequency of the filter, Hz


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def r_lowfilter(data):
    global r_pospub
    global order, fs, cutoff

    r_posXa.append( data.x )
    r_posYa.append( data.y )
    r_posZa.append( data.z )

    if len(r_posXa) > 40:
        del r_posXa[0]
        del r_posYa[0]
        del r_posZa[0]

        posXf = butter_lowpass_filter(r_posXa, cutoff, fs, order)
        posYf = butter_lowpass_filter(r_posYa, cutoff, fs, order)
        posZf = butter_lowpass_filter(r_posZa, cutoff, fs, order)

        posfilter = Point()

        posfilter.x = posXf[-1]
        posfilter.y = posYf[-1]
        posfilter.z = posZf[-1]

        r_pospub.publish(posfilter)

def l_lowfilter(data):
    global l_pospub
    global order, fs, cutoff

    l_posXa.append( data.x )
    l_posYa.append( data.y )
    l_posZa.append( data.z )

    if len(l_posXa) > 40:
        del l_posXa[0]
        del l_posYa[0]
        del l_posZa[0]

        posXf = butter_lowpass_filter(l_posXa, cutoff, fs, order)
        posYf = butter_lowpass_filter(l_posYa, cutoff, fs, order)
        posZf = butter_lowpass_filter(l_posZa, cutoff, fs, order)

        posfilter = Point()

        posfilter.x = posXf[-1]
        posfilter.y = posYf[-1]
        posfilter.z = posZf[-1]

        l_pospub.publish(posfilter)


def listener():
    global r_pospub, l_pospub
    rospy.init_node('lowpassfilter', anonymous=True)

    rospy.Subscriber("/camera/unfiltered/rightHipHand", Point, r_lowfilter)
    rospy.Subscriber("/camera/unfiltered/leftHipHand", Point, l_lowfilter)
    r_pospub = rospy.Publisher('/camera/rightHipHand', Point, queue_size=10)
    l_pospub = rospy.Publisher('/camera/leftHipHand', Point, queue_size=10)


    rospy.spin()

if __name__ == '__main__':
    listener()



