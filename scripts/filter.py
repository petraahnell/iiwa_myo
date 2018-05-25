#!/usr/bin/env python
import rospy
from scipy.signal import butter, lfilter
from scipy.signal import freqs
from geometry_msgs.msg import Point

global posXa, posYa, posZa

posXa = [0]
posYa = [0]
posZa = [0]

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def lowfilter(data):
    global pospub

    posXa.append( data.x )
    posYa.append( data.y )
    posZa.append( data.z )

    if len(posXa) > 40:
        del posXa[0]
        del posYa[0]
        del posZa[0]

        # Filter requirements.
        order = 3
        fs = 30.0       # sample rate, Hz
        cutoff = 0.8    # desired cutoff frequency of the filter, Hz

        posXf = butter_lowpass_filter(posXa, cutoff, fs, order)
        posYf = butter_lowpass_filter(posYa, cutoff, fs, order)
        posZf = butter_lowpass_filter(posZa, cutoff, fs, order)

        posfilter = Point()

        posfilter.x = posXf[-1]
        posfilter.y = posYf[-1]
        posfilter.z = posZf[-1]

        pospub.publish(posfilter)

    
def listener():
    global pospub
    rospy.init_node('lowpassfilter', anonymous=True)

    rospy.Subscriber("/camera/unfiltered/rightHipHand", Point, lowfilter)
    pospub = rospy.Publisher('/camera/rightHipHand', Point, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()


