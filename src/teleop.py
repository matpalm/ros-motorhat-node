#!/usr/bin/env python

# simple tele operation that translates key presses, e.g. 'w'
# to motorhat node messages. does simple interpolation ramp up
# of change between current value and target value.

#import readchar
import sys, select, tty, termios
import rospy
from std_msgs.msg import Int16MultiArray

RANGE = 150  # expect all node values to be in interval [-RANGE, RANGE]
STEP = 50    # interpolate between current & target at this rate.

def step(a, b):
    """ for each element in 'a' step it towards element in 'b'"""
    s = []
    for va, vb in zip(a, b):
        if va == vb:
            s.append(va)
        elif va < vb:
            s.append(va + STEP)
        else:
            s.append(va - STEP)
    return s

# setup a ros publisher and a method to send msgs to it.
publisher = rospy.Publisher('/robotZero/hat_cmd', Int16MultiArray, queue_size=1, latch=True)
rospy.init_node("teleop")
rate = rospy.Rate(100)
def pub(values):
    values = [-v for v in values]  # hack
    rospy.loginfo("publish [%s]" % values)
    msg = Int16MultiArray()
    msg.data = values
    publisher.publish(msg)

# set initial values to [0] and publish them.
current = [0] * 4
targets = [0] * 4
pub(current)

# set cbreak on stdin (recall original attr to restore later)
# this is required for the polling select on stdin.
original_t_attr = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())


while not rospy.is_shutdown():
    set_immediately = False
    try:
        # select for next char, timeout in 0.1s
        if select.select([sys.stdin], [], [], 0.1)[0] == [sys.stdin]:
            c = sys.stdin.read(1)
            if c== ' ':  # immediate stop
                set_immediately = True
                targets = [0] * 4
            elif c== 'w':  # forward
                targets = [RANGE] * 4
            elif c== 's':  # slow stop
                targets = [0] * 4
            elif c== 'x':  # backwards
                targets = [-RANGE] * 4
            elif c== 'a':  # skid steer left
                targets = [RANGE, -RANGE, RANGE, -RANGE]
            elif c== 'd':  # skip steer right
                targets = [-RANGE, RANGE, -RANGE, RANGE]
    except:
        # select error? assume ctrl-c during in blocking select
        # and completely bail now.
        break

    # move current values towards target if required.
    if current != targets:
        if set_immediately:
            current = targets
        else:
            current = step(current, targets)
        pub(current)

    # spin ros
    rate.sleep()

# pub stop and restore terminal settings
pub([0]*4)
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_t_attr)
