import cv2
import numpy as np
import time
import requests
from ast import Pass
from threading import Thread
from webcamvideostream import WebcamVideoStream
import matplotlib.pyplot as plt
import json
import random
import tkinter as Tk
from itertools import count
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

address = 'http://192.168.10.226:3000/'
pos = np.zeros((4, 3))
dt = 1
heading = 0
# Create figure for plotting
fig = plt.figure()
ax1 = fig.add_subplot(3, 1, 1)
ax2 = fig.add_subplot(3,1,2)
ax3 = fig.add_subplot(3,1,3)
xs = []
ys1 = []
ys2 = []
ys3 = []
ys4 = []
time_ = 0
start_time = time.perf_counter()
def runGetThread():
    Stop = False
    prevTime = time.time()
    time_ = 0

    while not Stop:
        if (time.time() - prevTime) > dt:
            time_ = time_ + dt
            req = requests.get(address + "agents/1")
            pReq = json.loads(req.text)
            dataPos  = pReq["position"]
            heading = dataPos[2] 
            prevTime = time.time()

# t1 = Thread(target = runGetThread)
# t1.daemon = False
# t1.start()


count_ = 0
def animate(i, xs, ys1, ys2, ys3,ys4):
    # time_ = time_ + dt
    time_ = time.perf_counter() -  start_time 
    req = requests.get(address + "agents/1")
    pReq = json.loads(req.text)
    dataPos  = pReq["position"]
    x = dataPos[0]
    y = dataPos[1]

    reqHeadingError = requests.get(address + "agentABC/1")
    pReqHeadingError = json.loads(reqHeadingError.text)
    dataHeadingError  = pReqHeadingError["value1"]
    HeadingError = dataHeadingError
    # print(heading)
    xs.append(time_)
    # error = (heading - 1.57)

    ys1.append(x)
    ys2.append(y)
    ys3.append(HeadingError*180/math.pi)
    ys4.append(dataPos[2]*180/math.pi)
    # Limit x and y lists to 20 items
    xs = xs[-200:]
    ys1 = ys1[-200:]
    ys2 = ys2[-200:]
    ys3 = ys3[-200:]
    ys4 = ys4[-200:]
    # Draw x and y lists
    ax1.clear()
    ax1.set_xlim([-1.5,1.5])
    ax1.set_ylim([-1.5,1.5])
    ax2.clear()
    ax3.clear()

    ax1.set_ylabel('m')
    ax1.set_xlabel('m')
    ax1.plot(ys1, ys2)
    ax1.set_title("2D Trajectory")
    # ax1.legend()


    ax2.plot(xs, ys3 ,label = 'heading error')
    ax2.set_ylabel('deg')
    ax2.legend()

    ax3.plot(xs,ys4, label = 'heading')
    ax3.set_ylabel('deg')
    ax3.legend()
    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)


# Set up plot to call animate() function periodically
time_ = 0
ani = animation.FuncAnimation(fig, animate, fargs=(xs,ys1,ys2,ys3,ys4), interval= 100)
plt.show()