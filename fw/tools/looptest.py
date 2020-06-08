#!/usr/bin/env python3

import random
import serial
import sys
import time
import threading
import struct

TOTAL = 256*1024
BLOCK_SIZE = 4096


ignore = True

def read_serial(port):
    prev = 0

    last = b""
    while True:
        data = port.read(4)
        if ignore:
            continue
    
        cur = last + data
        while len(cur) >= 4:
            val = struct.unpack("<I", cur[:4])[0]
            if val % 1024 == 0:
                print("%08x" % val)
            if val != prev and not ignore:
                print("Error, %08x instead of %08x" % (val, prev))
                raise Exception()
            prev = (val + 1) & 0xFFFFFFFF
            cur = cur[4:]

        last = cur


if __name__ == "__main__":
    with serial.Serial(sys.argv[1], baudrate=921600, timeout=0.5) as port:
        threading.Thread(target=read_serial, args=(port, ), daemon=True).start()

        time.sleep(1)
        ignore = False
        num = 0
        start = time.monotonic()
        for _ in range(int(TOTAL/BLOCK_SIZE)):
            buf = b""
            for i in range(int(BLOCK_SIZE/4)):
                buf += struct.pack("<I", num)
                #print("-> %08x" % num)
                num = (num +1) & 0xFFFFFFFF
            port.write(buf)
            #port.flush()
            #time.sleep(0.01)
        
        end = time.monotonic()
    
        time.sleep(1)
        print("Duration: %ss %s KB/s %s b/s" % ((end - start), (TOTAL/1024)/(end - start), (TOTAL*8)/(end - start)))
        print("num: %08x" % (num - 1))
