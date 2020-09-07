#!/usr/bin/env python
#coding=utf-8

import sys
import serial
import time
import numpy as np

import random

import plain_serial as ps

args = sys.argv
 
dev = serial.Serial(args[1], 9600, timeout=1.0)

cuart = ps.PlainSerial(dev)

def main():
    cuart.add_frame(ps.PlaneTwist())
    cuart.add_frame(ps.Float32())

    bit_list = ps.Bools()

    cuart.add_frame(bit_list)
    toggle = False
    while True:
        x = random.random()
        y = random.random()
        yaw = random.random()
        toggle = ~toggle
        bit_list.set(toggle, 4)
        cuart.send(0, ps.PlaneTwist(x,y,yaw))
        time.sleep(0.001)
        print(cuart.recv())
        cuart.send(1, ps.Float32(x))
        time.sleep(0.001)
        print(cuart.recv())
        cuart.send(2, bit_list)
        time.sleep(0.001)
        cuart.recv()
        print(bit_list.get(4))

if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        dev.close()
        sys.exit()
