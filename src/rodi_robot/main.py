#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rodi_robot.transport import Transport


class RodiRobot(object):

    ''' Robot class to add ROS support to RoDI'''

    def __init__(self):
        hostname = rospy.get_param('~hostname', '192.168.4.1')
        port = rospy.get_param('~port', '1234')
        self.timeout = rospy.get_param('~timeout', 2.0)

        self.transport = Transport(hostname, port)

        rospy.Subscriber('cmd_vel',
                         Twist,
                         self._cmd_vel_cb)
        self.pub = rospy.Publisher('ultrasound',
                                   Range,
                                   queue_size=1)

        self.rate = rospy.Rate(1)  # 1 Hz

        self.us_sensor = Range()
        self.us_sensor.radiation_type = 0  # ULTRASOUND
        self.us_sensor.header.frame_id = "/ultrasound"
        self.us_sensor.field_of_view = 0.52
        self.us_sensor.min_range = 0.2
        self.us_sensor.max_range = 1.0

        self.last_cmd_vel = rospy.Time.now()

    def _cmd_vel_cb(self, msg):
        self.last_cmd_vel = rospy.Time.now()
        if msg.angular.z == 0 and msg.linear.x == 0:
            self.transport.stop()
        elif msg.linear.x > 0:
            self.transport.move_forward()
        elif msg.linear.x < 0:
            self.transport.move_reverse()
        elif msg.angular.z > 0:
            self.transport.move_left()
        elif msg.angular.z < 0:
            self.transport.move_right()

    def run(self):
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.last_cmd_vel).secs >= self.timeout:
                self.transport.stop()
                self.last_cmd_vel = rospy.Time.now()
            try:
                range = self.transport.see()
                if range is not None:
                    self.us_sensor.range = float(range) / 100.0
                    self.us_sensor.header.stamp = rospy.Time.now()
                    self.pub.publish(self.us_sensor)
            except Exception as exception:
                rospy.logerr("getting sonar data failed: " + str(exception))

            self.rate.sleep()
