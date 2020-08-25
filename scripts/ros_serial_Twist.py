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

import serial
import time

from command_uart import CommandUart

from geometry_msgs.msg import Twist

rospy.init_node('plain_serial_Twist')
#接続先
port = rospy.get_param('~port')

dev = serial.Serial(port, 9600, timeout=1.0)
cuart = CommandUart(dev)

def got_request_cb(message):
    cuart.send(1, message.linear.x)
    time.sleep(0.005) #wait 5ms
    cuart.send(2, message.linear.y)
    time.sleep(0.005) #wait 5ms
    cuart.send(3, message.angular.z)
    time.sleep(0.005) #wait 5ms

def main():
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb)
    pub = rospy.Publisher('/plain_serial/odometry', Twist, queue_size=1)

    res = Twist()
    while not rospy.is_shutdown():
        result = cuart.recv()
        if result[0] == 1:
            res.linear.x = result[1]
            pub.publish(res)
        elif result[0] == 2:
            res.linear.y = result[1]
            pub.publish(res)
        elif result[0] == 3:
            res.angular.z = result[1]
            pub.publish(res)
        else:
            pass

if __name__ == '__main__':
    main()