#!/usr/bin/env python

from rospy import logdebug
from httplib import HTTPConnection


class Transport(object):

    def __init__(self, hostname='192.168.4.1', port='1234'):
        self.hostname = hostname
        self.port = port

    def send_command(self, params):
        request = "/" + "/".join(map(str, params))

        try:
            self.conn = HTTPConnection(self.hostname,
                                       port=self.port,
                                       timeout=0.5)
            self.conn.request("GET", request)
            response = self.conn.getresponse().read()
            self.conn.close()
            if len(response) == 0:
                return None
            return response
        except Exception as e:
            logdebug("the HTTP request " + request + " failed: " + str(e))
            return None

    def move_forward(self):
        self.send_command([3, 100, 100])

    def move_reverse(self):
        self.send_command([3, -100, -100])

    def move_left(self):
        self.send_command([3, -100, 100])

    def move_right(self):
        self.send_command([3, 100, -100])

    def stop(self):
        self.send_command([3, 0, 0])

    def see(self):
        return self.send_command([5])
