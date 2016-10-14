#!/usr/bin/env python
import atexit
import rospy
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray

def turn_off_motors():
  print "stop"
  send([0,0,0,0])
atexit.register(turn_off_motors)

D = 50

sonars = [0, 0, 0]
def callback(msg, idx):
  global sonars
  sonars[idx] = msg.range

cmd_pub = rospy.Publisher('/robotZero/hat_cmd',
                          Int16MultiArray, queue_size=1,
                          latch=True)
  
rospy.init_node('sonar_subscriber')
for i in range(3):
  rospy.Subscriber("/robotZero/sonar_%d" % i, Range,
                   callback, i)

def send(values):
    values = [-v for v in values]  # hack
    rospy.loginfo("publish [%s]" % values)
    msg = Int16MultiArray()
    msg.data = values
    cmd_pub.publish(msg)


r = rospy.Rate(5)
while not rospy.is_shutdown():
  print sonars
  if sonars[0] == sonars[1] and sonars[1] == sonars[2]:
    # either all 0 or all 30. shutdown
    send([0,0,0,0])
  else:
    am = np.argmax(sonars)
    print "am", am
    # TODO: not handling case of 2 being 30.0
    if am == 0:  # front
      send([D,D,D,D])
    elif am == 1:  # right
      send([-D,D,-D,D])
    else:
      assert am == 2  # left
      send([D,-D,D,-D])
  r.sleep()
