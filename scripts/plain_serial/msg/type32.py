#!/usr/bin/env python
#coding=utf-8

import struct

from .msg_base import StructMem

class Int32(StructMem):
    INT32 = 0
    MAX_INT32_SIZE = 4 #byte

    def __init__(self, val=0):
        super(Int32,self).__init__(self.INT32,self.MAX_INT32_SIZE)
        self.set(val)
    
    def set(self,val):
        self._val = val

    def write_bytes(self):
        return struct.pack('<i', self._val)

    def read_bytes(self, data):
        return struct.unpack('<i', data)[0]

class Float32(StructMem):
    FLOAT32 = 1
    MAX_FLOAT32_SIZE = 4 #byte

    def __init__(self, val=0):
        super(Float32,self).__init__(self.FLOAT32,self.MAX_FLOAT32_SIZE)
        self.set(val)
    
    def set(self,val):
        self._val = val

    def write_bytes(self):
        return struct.pack('<f', self._val)

    def read_bytes(self, data):
        return struct.unpack('<f', data)[0]
