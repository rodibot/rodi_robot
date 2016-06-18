#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from transport import Transport

rospy.init_node('rodi_ws_node')

transport = Transport()

def callback(msg):
    if msg.angular.z == 0 and msg.linear.x == 0:
        transport.stop()
        return

    if msg.linear.x > 0:
        transport.move_forward()
        return

    if msg.linear.x < 0:
        transport.move_reverse()
        return

    if msg.angular.z > 0:
        transport.move_left()
        return

    if msg.angular.z < 0:
        transport.move_right()
        return

sub = rospy.Subscriber('cmd_vel', Twist, callback)

rospy.spin()
