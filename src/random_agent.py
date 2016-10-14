#!/usr/bin/env python

# simple tele operation that translates key presses, e.g. 'w'
# to motorhat node messages. does simple interpolation ramp up
# of change between current value and target value.

#import readchar
import sys, select, tty, termios
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

# setup a ros publisher and a method to send msgs to it.
publisher = rospy.Publisher('/robotZero/hat_cmd', Int16MultiArray,
                            queue_size=1, latch=True)
rospy.init_node("teleop")

def pub(values):
    values = [-v for v in values]  # hack
    rospy.loginfo("publish [%s]" % values)
    msg = Int16MultiArray()
    msg.data = values
    publisher.publish(msg)

# set cbreak on stdin (recall original attr to restore later)
# this is required for the polling select on stdin.
original_t_attr = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    d = np.random.randint(0, 7, size=4)
    d -= 3
    d *= 50
    pub(d)
    # spin ros
    rate.sleep()

# pub stop and restore terminal settings
pub([0]*4)
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_t_attr)
