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
import struct
import time

from geometry_msgs.msg import Twist

class CommandUart:
    DATA_SIZE =8

    #メッセージの型識別
    INT32       =chr(0)  
    FLOAT32     =chr(1)
    BOOL        =chr(2)

    #制御用のcharacter
    HEADER      =':'
    END         ='\n'

    def __init__(self, dev):
        self.uart = dev
        time.sleep(1)

    def send(self, id, val):
        frame = self.HEADER+chr(id)
        body = 0
        if isinstance(val, float):
            body = struct.pack('<f', val)
            frame += self.FLOAT32+body
        elif isinstance(val, int):
            body = struct.pack('<i', val)
            frame += self.INT32+body

        data_sum = sum(ord(i) for i in frame)+ord(self.END)
        frame += chr(data_sum & 0xFF)+self.END

        for i in frame :
            self.uart.write(str(i))

    def recv(self):
        got_char = self.uart.read()
        if got_char == self.HEADER:
            data=""
            check_sum = ord(self.HEADER)
            counter = 1
            while got_char != self.END:
                if self.DATA_SIZE < counter : return -1, 0
                got_char = self.uart.read()
                check_sum += ord(got_char)
                data = data + got_char
                counter += 1
            
            if len(data) != self.DATA_SIZE:
                return -1, 0

            id = ord(data[0])
            msg_id = ord(data[1])
            check_sum = (check_sum - ord(data[-2])) & 0xFF
            data_sum = ord(data[-2])
            data = data[2:-2]

            if (check_sum - data_sum) == 0:
                if msg_id == ord(self.INT32):
                    return id, struct.unpack('<i', data)[0]
                elif msg_id == ord(self.FLOAT32):
                    return id, struct.unpack('<f', data)[0]
                else:
                    return -1, 0
            else:
                return -1, 0
        else:
            return -1, 0

dev = serial.Serial('/dev/ttyS3', 9600, timeout=1.0)
cuart = CommandUart(dev)

def got_request_cb(message):
    cuart.send(1, message.linear.x)
    time.sleep(0.005) #wait 5ms
    cuart.send(2, message.linear.y)
    time.sleep(0.005) #wait 5ms
    cuart.send(3, message.angular.z)
    time.sleep(0.005) #wait 5ms

if __name__ == '__main__':
    rospy.init_node('plain_serial_Twist')
    sub = rospy.Subscriber('/plain_serial/cmd_vel', Twist, got_request_cb)
    pub = rospy.Publisher('/plain_serial/odom_vel', Twist, queue_size=1)

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
