#! /usr/bin/env python
#encoding=utf-8
#
# File:     imu_msg.py
# 
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import struct

from .msg_base import StructMem

class ImuMsg(StructMem):
    IMU_MSG = 127
    MAX_P_IMU_MSG_SIZE = 34 #byte

    def __init__(self):
        super(ImuMsg,self).__init__(self.IMU_MSG,self.MAX_P_IMU_MSG_SIZE)
        self.pose = self.Pose()
        self.acc = self.Acc()
        self.gyro = self.Gyro()

    class Pose:
        def __init__(self):
            self.x = 0
            self.y = 0
            self.z = 0
            self.w = 0

    class Acc:
        def __init__(self):
            self.x = 0
            self.y = 0
            self.z = 0

    class Gyro:
        def __init__(self):
            self.x = 0
            self.y = 0
            self.z = 0

    def write_bytes(self):
        return struct.pack('<ffffhhhfff',   self.pose.x,self.pose.y,self.pose.z,self.pose.w,
                                            self.acc.x,self.acc.y,self.acc.z,
                                            self.gyro.x,self.gyro.y,self.gyro.z)

    def read_bytes(self, data):
        return struct.unpack('<ffffhhhfff', data)[0:10]
