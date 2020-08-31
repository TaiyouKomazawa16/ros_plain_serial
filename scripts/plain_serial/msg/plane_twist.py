#!/usr/bin/env python
#coding=utf-8

import struct

from .msg_base import StructMem

class PlaneTwist(StructMem):
    PLANE_TWIST = 2
    MAX_P_TWIST_SIZE = 12 #byte

    def __init__(self, x=0, y=0, yaw=0):
        super(PlaneTwist,self).__init__(self.PLANE_TWIST,self.MAX_P_TWIST_SIZE)
        self.set(x,y,yaw)
    
    def set(self,x,y,yaw):
        self._x = x
        self._y = y
        self._yaw = yaw

    def write_bytes(self):
        return struct.pack('<fff', self._x,self._y,self._yaw)

    def read_bytes(self, data):
        return struct.unpack('<fff', data)[0:3]
