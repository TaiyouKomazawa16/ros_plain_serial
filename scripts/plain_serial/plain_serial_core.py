#!/usr/bin/env python
#coding=utf-8

import sys
import serial
import time

from .msg.msg_base import StructMem

class PlainSerial:
    FRAME_LEN = 4

    #制御用のcharacter
    HEADER      =':'
    END         ='\n'

    def __init__(self, dev):
        self.uart = dev
        time.sleep(1)

    def _write(self, data):
        self.uart.write(str(data))
        return sum(ord(i) for i in data)

    def _read(self, size = 1):
        result = self.uart.read(size)
        return sum(ord(i) for i in result), result

    def send(self, id, message):
        if issubclass(type(message), StructMem) == False:
            return #error!

        data_sum = 0
        data_sum += self._write(self.HEADER)
        data_sum += self._write(chr(id))
        data_sum += self._write(chr(message.msg_id()))
        data_sum += self._write(message.write_bytes())
        data_sum += ord(self.END)
        self._write(chr(data_sum & 0xFF)+self.END)

    def recv(self, message):
        if issubclass(type(message), StructMem) == False:
            return #error!

        [check_sum, got_char] = self._read()
        if got_char == self.HEADER:
            tmp = self._read(message.size()+self.FRAME_LEN)
            check_sum += tmp[0]
            data = tmp[1]
            if data[-1] != self.END:
                return -1, 0

            id = ord(data[0])
            msg_id = ord(data[1])
            check_sum = (check_sum - ord(data[-2])) & 0xFF
            data_sum = ord(data[-2])
            data = data[2:-2]

            if (check_sum - data_sum) == 0:
                if msg_id == message.msg_id():
                    return id, message.read_bytes(data)
                else:
                    return -1, 0
            else:
                return -1, 0
        else:
            return -1, 0
