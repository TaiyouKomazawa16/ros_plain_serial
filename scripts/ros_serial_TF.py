#! /usr/bin/env python
#encoding=utf-8
#
# File:     ros_serial_TF.py
# 
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import sys,math
import rospy

import serial
import time

import tf

from command_uart import CommandUart

from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from nav_msgs.msg import Odometry

rospy.init_node('plain_serial_TF')
#接続先
port = rospy.get_param('~port')

dev = serial.Serial(port, 9600, timeout=1.0)
cuart = CommandUart(dev)


class CalcOdometry():

    def __init__(self):
        self.l_t = rospy.Time.now()
        self.l_p = Twist()
        self.tf_bc_odom = tf.TransformBroadcaster()

    def _get_vel(self, twist, c_t):
        dt = self.c_t.to_sec() - self.l_t.to_sec()
        diff = Twist()
        diff.linear.x = (twist.linear.x - self.l_p.linear.x) / dt
        diff.linear.y = (twist.linear.y - self.l_p.linear.y) / dt
        diff.angular.z = (twist.angular.z - self.l_p.angular.z) / dt
        l_p = twist

        return diff

    def calc_tf(self, twist):
        self.c_t = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(0,0, twist.angular.z)
        self.tf_bc_odom.sendTransform((twist.linear.x,twist.linear.y,0.0),q,self.c_t,"base_link","odom")

        odom = Odometry()

        #tfタグ
        odom.header.stamp = self.c_t
        odom.header.frame_id =  "odom"
        odom.child_frame_id =   "base_link"
        #現在のオドメトリ
        odom.pose.pose.position = Point(twist.linear.x, twist.linear.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)
        #現在の速度ベクトル
        vel = self._get_vel(twist, self.c_t)
        odom.twist.twist = vel

        self.l_t = self.c_t

        return odom



def main():
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb)
    pub = rospy.Publisher('/plain_serial/odometry', Odometry, queue_size=10)

    res = Twist()
    rate = rospy.Rate(10)

    codom = CalcOdometry()

    while not rospy.is_shutdown():
        result = cuart.recv()
        if result[0] == 1:
            res.linear.x = result[1]
        elif result[0] == 2:
            res.linear.y = result[1]
        elif result[0] == 3:
            res.angular.z = result[1]
        else:
            pass

        pub.publish(codom.calc_tf(res))
        rate.sleep()

def got_request_cb(message):
    cuart.send(1, message.linear.x)
    time.sleep(0.005) #wait 5ms
    cuart.send(2, message.linear.y)
    time.sleep(0.005) #wait 5ms
    cuart.send(3, message.angular.z)
    time.sleep(0.005) #wait 5ms

if __name__ == '__main__':
    main()
