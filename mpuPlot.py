#!/usr/bin/python3
import secrets
import numpy as np
import matplotlib.pyplot as plt
import socket
from matplotlib.animation import FuncAnimation
import sys
import re
import time
import struct
import threading

# need to be the same than pico mpu6050
#FSAMP   SAMPLE FREQUENCY in HZ
FSAMP = 500

#SAMP  number of sample taken by the FFT
SAMP = 512

#minimum threshold to modify the chart (0.0 = disable)
minThreshold = 0.0

UDP_PORT=6001
ServerSocket = socket.socket(family=socket.AF_INET, type = socket.SOCK_DGRAM)

ServerSocket.bind(('',UDP_PORT))

lock = threading.Lock()
numbers = (0)*(SAMP//2)
numbers_t = (0)*(SAMP//2)
maxPeak = 0

#define  RECEIVER_IP  NULL
#define  RECEIVER_PORT 6001


CycleFreq =  (FSAMP/SAMP)
Tcount=0
ExitFlag=False


def GetFFT():
   global Tcount
   global numbers_t
   global ServerSocket
   global ExitFlag
   global maxPeak
   print("thread start")
   while not ExitFlag:
       info = ServerSocket.recvfrom(1024)
       ln = len(info[0])
       if ln < SAMP:
           continue
       numbers_th = struct.unpack('H'*(SAMP//2),info[0][0:SAMP])
       if len(numbers_th) != (SAMP//2):
           continue
       freq = CycleFreq*numbers_th[0]
       nidx = numbers_th[0]
       if nidx >= (SAMP//2):
           continue
       print("Tcount",Tcount," Freq:",freq," value:",numbers_th[nidx])
       lock.acquire()
       maxPeak= numbers_th[0]
       numbers_t = (0.001/(SAMP /2)) * np.array(numbers_th) # from milli g to g
       if numbers_t[maxPeak] > minThreshold :
            Tcount+=1
       lock.release()
   print("Thread close")



xdata= np.arange(0,(SAMP//2)*CycleFreq,CycleFreq)

plt.xticks(xdata)

CurrentCount=0;

thread1= threading.Thread(target=GetFFT)
thread1.start()
try:
    while True:

        lock.acquire()
        if Tcount==CurrentCount:
            lock.release()
            continue
        numbers=numbers_t
        CurrentCount=Tcount
        lock.release()
        print("show ",CurrentCount)
        plt.clf()
        plt.plot(xdata,numbers,color=(0.2, 0.4, 0.6, 0.6))
        plt.title("Freq:{:.1f}Hz max:{:.3f}g".format(xdata[maxPeak],numbers[maxPeak]))
        plt.axis([0,(SAMP//2),0,2])
        plt.pause(0.0001)
except KeyboardInterrupt:
       pass
ExitFlag=True
print("done")

