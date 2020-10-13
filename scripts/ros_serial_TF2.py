#! /usr/bin/env python
#encoding=utf-8
#
# File:     ros_serial_TF2.py
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
name = rospy.get_param('~device_name')
port_name = rospy.get_param('~port_name', "ttyUSB")

cuart = ps.PlainSerial(name, tty_head=port_name)

cmds = ps.Bools()

mutex = Lock()


class CalcOdometry():

    def __init__(self):
        self.l_t = rospy.Time.now()
        self.tf_bc_odom = tf2_ros.TransformBroadcaster()
        self.x = 0.0
        self.y = 0.0
        self.lyaw = 0.0

    def calc_tf(self, res):
        self.c_t = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(0,0, res.angular.z)
        t = TransformStamped()
        odom = Odometry()

        dt = self.c_t.to_sec() - self.l_t.to_sec()
        self.x += res.linear.x * dt
        self.y += res.linear.y * dt
        vyaw = (res.angular.z - self.lyaw) / dt
        self.lyaw = res.angular.z

        #tf2タグ
        t.header.stamp = self.c_t
        t.header.frame_id =     "ps_odom"
        t.child_frame_id =      "base_link"
        #現在のオドメトリ
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
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
        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q)
        #現在の速度ベクトル
        odom.twist.twist.linear.x = res.linear.x
        odom.twist.twist.linear.y = res.linear.y
        odom.twist.twist.angular.z = vyaw

        self.l_t = self.c_t

        return odom


def main():
    srv = rospy.Service('/plain_serial/sys_cmd', BoolCommand, got_command_cb)
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb, queue_size=10)
    pub = rospy.Publisher('/plain_serial/odometry', Odometry, queue_size=10)

    ctrl_rate = rospy.Rate(50)   #50hz

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
        ctrl_rate.sleep()

    cuart.close()


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
        time.sleep(0.005)
    mutex.release()

    return True


if __name__ == '__main__':
    main()
