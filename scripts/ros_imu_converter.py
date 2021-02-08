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

import numpy as np

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

    imu_pub = rospy.Publisher('/plain_serial/imu', Imu, queue_size=10)

    ctrl_rate = rospy.Rate(23)   #23hz

    cuart.add_frame(ps.ImuMsg())

    imu_frame_id = rospy.get_param("~frame_id", "imu_link")

    # NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
    linear_acceleration_stdev_ = rospy.get_param("linear_acceleration_stdev", (400 / 1000000.0) * 9.807 )

    # Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
    angular_velocity_stdev_ = rospy.get_param("angular_velocity_stdev", 0.05 * (math.pi / 180.0))

    # 1 degree for pitch and roll
    pitch_roll_stdev_ = rospy.get_param("pitch_roll_stdev", 1.0 * (math.pi / 180.0))

    # 5 degrees for yaw
    yaw_stdev_ = rospy.get_param("yaw_stdev", 5.0 * (math.pi / 180.0))

    angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_
    linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_
    yaw_covariance = yaw_stdev_ * yaw_stdev_

    SAMPLE_NUM = 40
    data_head = 0
    rot_array = np.zeros([SAMPLE_NUM, 3])
    acc_array = np.zeros([SAMPLE_NUM, 3])

    imu_msg.header.frame_id = imu_frame_id

    imu_msg.orientation_covariance[0] = pitch_roll_covariance
    imu_msg.orientation_covariance[4] = pitch_roll_covariance
    imu_msg.orientation_covariance[8] = yaw_covariance

    #imu_msg.angular_velocity_covariance[0] = angular_velocity_covariance
    #imu_msg.angular_velocity_covariance[4] = angular_velocity_covariance
    #imu_msg.angular_velocity_covariance[8] = angular_velocity_covariance

    imu_msg.linear_acceleration_covariance[0] = linear_acceleration_covariance
    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_covariance
    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_covariance

    rospy.loginfo("Ready..")

    while not rospy.is_shutdown():
        sensor_data_time = rospy.Time.now()
        result = cuart.recv()
        if result[0] == 0:
            imu_msg.orientation.x = result[1][0]
            imu_msg.orientation.y = result[1][1]
            imu_msg.orientation.z = result[1][2]
            imu_msg.orientation.w = result[1][3]

            imu_msg.linear_acceleration.x = result[1][4] * 1/16384.0 * 9.80665
            imu_msg.linear_acceleration.y = result[1][5] * 1/16384.0 * 9.80665
            imu_msg.linear_acceleration.z = result[1][6] * 1/16384.0 * 9.80665

            #imu_msg.angular_velocity.x = result[1][7]
            #imu_msg.angular_velocity.y = result[1][8]
            #imu_msg.angular_velocity.z = result[1][9]

            e = tf_conversions.transformations.euler_from_quaternion((  result[1][0], 
                                                            result[1][1], 
                                                            result[1][2], 
                                                            result[1][3]))
            rot_array[data_head] = [e[0], e[1], e[2]]
            rot_cov = np.cov(rot_array, rowvar=False, bias=True)
            imu_msg.orientation_covariance = rot_cov.reshape((1, 9))[0]

            acc_array[data_head] = [imu_msg.linear_acceleration.x, 
                                    imu_msg.linear_acceleration.y, 
                                    imu_msg.linear_acceleration.z]
            acc_cov = np.cov(acc_array, rowvar=False, bias=True)
            imu_msg.linear_acceleration_covariance = acc_cov.reshape((1, 9))[0]

            imu_msg.header.stamp = sensor_data_time
            imu_pub.publish(imu_msg)
        ctrl_rate.sleep()

    cuart.close()

if __name__ == '__main__':
    main()
