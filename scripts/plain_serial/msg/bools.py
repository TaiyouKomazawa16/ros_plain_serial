#!/usr/bin/env python
#coding=utf-8

import struct

from .msg_base import StructMem

class Bools(StructMem):
    BOOLS = 3
    MAX_BOOLS_SIZE = 1 #byte

    _send_data = 0
    _data = 0

    def __init__(self):
        super(Bools,self).__init__(self.BOOLS,self.MAX_BOOLS_SIZE)
    
    def set(self, val, bit):
        self._send_data = self._bit_write(self._send_data, bit, val)
    
    def get(self, bit):
        return self._bit_read(self._data, bit)


    def write_bytes(self):
        return chr(self._send_data)

    def read_bytes(self, data):
        self._data = ord(data[0])
        return self._data

    def _bit_write(self, value, bit, bitvalue): 
        if bool(bitvalue) : return (value) | (1 << bit)
        else : return (value) & ~(1 << bit)

    def _bit_read(self, value, bit):
        return bool((value >> bit) & 0x01)

