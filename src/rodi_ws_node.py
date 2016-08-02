#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
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
pub = rospy.Publisher('ultrasound', Range, queue_size=1)

rate = rospy.Rate(2) # 2 Hz

range = Range()
range.radiation_type = 0 # ULTRASOUND
range.header.frame_id = "/ultrasound"
range.field_of_view = 0.52;
range.min_range = 0.2;
range.max_range = 4.0;

while not rospy.is_shutdown():
    try:
        range.range = float(transport.see()) / 100.0
        range.header.stamp = rospy.Time.now()
        pub.publish(range)
    except Exception as e:
        rospy.logerr("getting sonar data failed: " + str(e))

    rate.sleep()

