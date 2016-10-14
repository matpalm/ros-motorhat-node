#!/usr/bin/env python
import atexit
import rospy
import numpy as np
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray

cmd_pub = rospy.Publisher('/robotZero/hat_cmd',
                          Int16MultiArray, queue_size=1,
                          latch=True)
msg = Int16MultiArray()
msg.data = [0,0,0,0]
rospy.init_node('stop')
cmd_pub.publish(msg)
