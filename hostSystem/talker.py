import cv2
import numpy as np
import time
import requests
from ast import Pass
from threading import Thread
from webcamvideostream import WebcamVideoStream
address = 'http://192.168.10.226:3000/'
pos = np.zeros((4, 3))
dt = 0.05


def runPutThread():
    Stop = False
    prevTime = time.time()
    while not Stop:
        if (time.time() - prevTime) > dt:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                Stop = True
                break
            prevTime = time.time()
            for i in range(3):
                pos[i+1][0] = np.random.rand(1,1)
                pos[i+1][1] = np.random.rand(1,1)
                pos[i+1][2] = np.random.rand(1,1)
            data = {"id": 1, "pos":pos[1:].tolist()}
            requests.put(address + "allPos/1", json=data)
            print('put protocol is completed')
t1 = Thread(target = runPutThread)
t1.daemon = False
t1.start()

