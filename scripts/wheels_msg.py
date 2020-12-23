#! /usr/bin/env python
#encoding=utf-8
#
# File:     wheels_msg.py
# 
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import struct

from plain_serial.msg.msg_base import StructMem

class WheelsMsg(StructMem):
    WHEELS_MSG = 128
    MAX_P_WHEELS_MSG_SIZE = 16 #byte

    def __init__(self):
        super(WheelsMsg,self).__init__(self.WHEELS_MSG,self.MAX_P_WHEELS_MSG_SIZE)
        self.wheels = self.Wheel()

    class Wheel:
        def __init__(self):
            self.fl = 0
            self.rl = 0
            self.rr = 0
            self.fr = 0

    def write_bytes(self):
        return struct.pack('<ffff', self.wheels.fl,self.wheels.rl,self.wheels.rr,self.wheels.fr)

    def read_bytes(self, data):
        return struct.unpack('<ffff', data)[0:4]
