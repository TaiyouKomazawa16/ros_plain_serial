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
import wheels_msg

from ros_plain_serial.srv import BoolCommand

from std_msgs.msg import Float32MultiArray
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
        self.odom = Odometry()

        #covariance matrix
        #   0   1   2   3   4   5
        #   6   7   8   9   10  11
        #   12  13  14  15  16  17
        #   18  19  20  21  22  23
        #   24  25  26  27  28  29
        #   30  31  32  33  34  35
        self.odom.pose.covariance[0] = 0.003    #pose covariance xx
        self.odom.pose.covariance[7] = 0.003    #pose covariance yy
        self.odom.pose.covariance[35] = 0.10    #pose covariance yawyaw
        self.odom.twist.covariance[0] = 0.003   #vel covariance xx
        self.odom.twist.covariance[7] = 0.003   #vel covariance yy
        self.odom.twist.covariance[35] = 0.50   #vel covariance yawyaw

    def calc_tf(self, res):
        self.c_t = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(0,0, res.angular.z)
        t = TransformStamped()

        dt = self.c_t.to_sec() - self.l_t.to_sec()
        self.x += res.linear.x * dt
        self.y += res.linear.y * dt
        vyaw = (res.angular.z - self.lyaw) / dt
        self.lyaw = res.angular.z

        #tf2タグ
        t.header.stamp = self.c_t
        t.header.frame_id =     "odom"
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
        self.odom.header.stamp = self.c_t
        self.odom.header.frame_id =  "odom"
        self.odom.child_frame_id =   "base_link"
        #現在のオドメトリ
        self.odom.pose.pose.position = Point(self.x, self.y, 0)
        self.odom.pose.pose.orientation = Quaternion(*q)
        #現在の速度ベクトル
        self.odom.twist.twist.linear.x = res.linear.x
        self.odom.twist.twist.linear.y = res.linear.y
        self.odom.twist.twist.angular.z = vyaw

        self.l_t = self.c_t

        return self.odom


def main():
    srv = rospy.Service('/plain_serial/sys_cmd', BoolCommand, got_command_cb)
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb, queue_size=10)
    pub_odom = rospy.Publisher('/plain_serial/odometry', Odometry, queue_size=10)
    pub_wheel = rospy.Publisher('/plain_serial/wheels_vel', Float32MultiArray, queue_size=10)


    ctrl_rate = rospy.Rate(60)   #60hz

    codom = CalcOdometry()
    cuart.add_frame(ps.PlaneTwist())
    cuart.add_frame(cmds)
    cuart.add_frame(wheels_msg.WheelsMsg())

    odom = Twist()
    wheels_vel = Float32MultiArray()

    rospy.loginfo("Ready..")

    while not rospy.is_shutdown():
        result = cuart.recv()
        if result[0] == 0:
            odom.linear.x = result[1][0]
            odom.linear.y = result[1][1]
            odom.angular.z = result[1][2]
            pub_odom.publish(codom.calc_tf(odom))
            
        elif result[0] == 2:
            wheels_vel.data = result[1]
            pub_wheel.publish(wheels_vel)
        ctrl_rate.sleep()

    cmds.set(True, 1)
    mutex.acquire(1)
    for i in range(4):
        cuart.send(1, cmds)
        time.sleep(0.005)
    mutex.release()

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
