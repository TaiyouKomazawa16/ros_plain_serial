#encoding=utf-8
#
# File:     command_uart.py
# 
# 本ファイルではシリアル通信の受送信処理を行います。 
#
# This file is distributed under the MIT license. See "LICENSE" for text.
# 
# Author:   TaiyouKomazawa
#

import struct
import time
import serial

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
            while self.DATA_SIZE >= counter:
                got_char = self.uart.read()
                check_sum += ord(got_char)
                data = data + got_char
                counter += 1
            
            if data[-1] != self.END:
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
