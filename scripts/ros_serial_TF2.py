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
from threading import Lock

import serial
import time

import tf_conversions
import tf2_ros

import plain_serial as ps

from ros_plain_serial.srv import BoolCommand

from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from nav_msgs.msg import Odometry


rospy.init_node('plain_serial_TF2')
#接続先
port = rospy.get_param('~port')

dev = serial.Serial(port, 9600, timeout=1.0)
cuart = ps.PlainSerial(dev)

cmds = ps.Bools()

mutex = Lock()


class CalcOdometry():

    def __init__(self):
        self.l_t = rospy.Time.now()
        self.l_p = Twist()
        self.tf_bc_odom = tf2_ros.TransformBroadcaster()

    def _get_vel(self, twist, c_t):
        dt = self.c_t.to_sec() - self.l_t.to_sec()
        diff = Twist()
        diff.linear.x = (twist.linear.x - self.l_p.linear.x) / dt
        diff.linear.y = (twist.linear.y - self.l_p.linear.y) / dt
        diff.angular.z = (twist.angular.z - self.l_p.angular.z) / dt
        self.l_p = twist

        return diff

    def calc_tf(self, twist):
        self.c_t = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(0,0, twist.angular.z)
        t = TransformStamped()
        odom = Odometry()

        #tf2タグ
        t.header.stamp = self.c_t
        t.header.frame_id =     "ps_odom"
        t.child_frame_id =      "base_link"
        #現在のオドメトリ
        t.transform.translation.x = twist.linear.x
        t.transform.translation.y = twist.linear.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_bc_odom.sendTransform(t)

        #Odometryタグ
        odom.header.stamp = self.c_t
        odom.header.frame_id =  "ps_odom"
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
    srv = rospy.Service('/plain_serial/sys_cmd', BoolCommand, got_command_cb)
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb)
    pub = rospy.Publisher('/plain_serial/odometry', Odometry, queue_size=10)

    res = Twist()
    rate = rospy.Rate(50)   #50hz

    codom = CalcOdometry()
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

            pub.publish(codom.calc_tf(res))
        rate.sleep()


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
    for i in range(srv_req.retries):
        cuart.send(1, cmds)
    mutex.release()

    return True


if __name__ == '__main__':
    main()
