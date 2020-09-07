#! /usr/bin/env python
#encoding=utf-8
#
# File:     ros_serial_Twist.py
# 
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import sys,math
import rospy
from threading import Lock

import serial
import time

import plain_serial as ps

from geometry_msgs.msg import Twist

from ros_plain_serial.srv import BoolCommand

rospy.init_node('plain_serial_Twist')
#接続先
port = rospy.get_param('~port')

dev = serial.Serial(port, 9600, timeout=1.0)
cuart = ps.PlainSerial(dev)

cmds = ps.Bools()

mutex = Lock()

def got_request_cb(message):
    x = message.linear.x
    y = message.linear.y
    thr = message.angular.z
    mutex.acquire(1)
    cuart.send(0, ps.PlaneTwist(x,y,thr))
    mutex.release()

def got_command_cb(srv_req):
    rospy.loginfo("Get command.")
    cmds.set(srv_req.cmd, srv_req.bit)
    mutex.acquire(1)
    cuart.send(1, cmds)
    mutex.release()

    return True

def main():
    srv = rospy.Service('/plain_serial/sys_cmd', BoolCommand, got_command_cb)
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb)
    pub = rospy.Publisher('/plain_serial/odometry', Twist, queue_size=1)

    cuart.add_frame(ps.PlaneTwist())
    cuart.add_frame(cmds)

    res = Twist()

    rospy.loginfo("Ready..")

    while not rospy.is_shutdown():
        result = cuart.recv()
        if result[0] == 0:
            res.linear.x = result[1][0]
            res.linear.y = result[1][1]
            res.angular.z = result[1][2]
            pub.publish(res)

if __name__ == '__main__':
    main()