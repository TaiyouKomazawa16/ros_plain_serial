#! /usr/bin/env python
#encoding=utf-8
#
# File:     ros_imu_converter.py
# 
# Ros imu converter for mpu6050.
#
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import sys,math
import rospy

import serial

import tf_conversions
import tf2_ros

import plain_serial as ps

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


rospy.init_node('ros_imu_converter')
#接続先
name = rospy.get_param('~device_name')
port_name = rospy.get_param('~port_name', "ttyUSB")

cuart = ps.PlainSerial(name, tty_head=port_name)

def main():
    imu_msg = Imu()
    res = Quaternion()

    imu_pub = rospy.Publisher('/plain_serial/imu', Imu, queue_size=10)

    ctrl_rate = rospy.Rate(200)   #200hz

    cuart.add_frame(ps.Quaternion())

    linear_acceleration_stdev = rospy.get_param("~linear_acceleration_stdev", 0.06)
    angular_velocity_stdev = rospy.get_param("~angular_velocity_stdev", 0.005)
    imu_frame_id = rospy.get_param("~frame_id", "imu_link")

    linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev
    angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev


    imu_msg.header.frame_id = imu_frame_id

    imu_msg.orientation_covariance[0] = 0.1
    imu_msg.orientation_covariance[4] = 0.1
    imu_msg.orientation_covariance[8] = 0.1


    #Angular velocity entries are not filled, so set to -1.0
    imu_msg.angular_velocity_covariance[0] = 0.0
    imu_msg.angular_velocity_covariance[4] = angular_velocity_cov
    imu_msg.angular_velocity_covariance[8] = angular_velocity_cov

    #Linear acceleration entries are not filled, so set to -1.0
    imu_msg.linear_acceleration_covariance[0] = 0.0
    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_cov
    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_cov

    rospy.loginfo("Ready..")

    while not rospy.is_shutdown():
        sensor_data_time = rospy.Time.now()
        result = cuart.recv()
        if result[0] == 0:
            res.x = result[1][0]
            res.y = result[1][1]
            res.z = result[1][2]
            res.w = result[1][3]

            imu_msg.header.stamp = sensor_data_time
            imu_msg.orientation=res
            imu_pub.publish(imu_msg)
        ctrl_rate.sleep()

if __name__ == '__main__':
    main()
