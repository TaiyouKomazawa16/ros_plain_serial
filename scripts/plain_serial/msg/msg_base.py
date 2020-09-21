#! /usr/bin/env python
#encoding=utf-8
#
# File:     msg_base.py
# 
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

class StructMem(object):
    def __init__(self, msg_id, size):
        self._msg_id = msg_id
        self._size = size
    
    def msg_id(self):
        return self._msg_id
    def size(self):
        return self._size

    def write_bytes(self):  #virtual function
        pass

    def read_bytes(self, data): #virtual function
        pass
