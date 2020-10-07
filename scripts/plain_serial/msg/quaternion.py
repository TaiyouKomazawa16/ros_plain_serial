#! /usr/bin/env python
#encoding=utf-8
#
# File:     quaternion.py
# 
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import struct

from .msg_base import StructMem

class Quaternion(StructMem):
    QUATERNION = 4
    MAX_P_QUATERNION = 16 #byte

    def __init__(self, x=0, y=0, z=0, w=0):
        super(Quaternion,self).__init__(self.QUATERNION,self.MAX_P_QUATERNION)
        self.set(x,y,z,w)
    
    def set(self,x,y,z,w):
        self._x = x
        self._y = y
        self._z = z
        self._w = w

    def write_bytes(self):
        return struct.pack('<ffff', self._x,self._y,self._z,self._w)

    def read_bytes(self, data):
        return struct.unpack('<ffff', data)[0:4]
