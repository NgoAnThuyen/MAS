from __future__ import division
import os
from tkinter import *
import numpy as np
import requests


address = 'http://192.168.10.226:3000/'

class ControlApp(Tk, object):

   

    def __init__(self):
        self.speed = 0
        self.rotate = 0
        super(ControlApp, self).__init__()
        self.moving = False
        # Set up the interface
        self.geometry("100x100")
        self.bind("<KeyPress>", self.keydown)
        #self.bind("<KeyRelease>", self.keyup)
        self.sensorText = StringVar()
        self.sensorLabel = Label(self, textvariable=self.sensorText)
        self.sensorLabel.pack()
        self.mainloop()

    def display_sensor_data(self, msg):
        s = "Left: " + str(msg.leftRange) + "\nRight: " + str(msg.rightRange)
        self.sensorText.set(s)

    def keydown(self, event):
        # On/off actuators
        if event.keysym == "Up":
            self.speed = self.speed + 50
            self.speed = min(max(self.speed,-255),255)

        elif event.keysym == "Down":
            self.speed = self.speed - 50
            self.speed = min(max(self.speed,-255),255)

        elif event.keysym == "Left":
            self.rotate = self.rotate - 30
            self.rotate = min(max(self.rotate,-255),255)

        elif event.keysym == "Right":
            self.rotate = self.rotate + 30
            self.rotate = min(max(self.rotate,-255),255)

        elif event.keysym == "q":
            self.rotate = 0

        elif event.keysym == "s":
            self.rotate = 0
            self.speed = 0

        msg = 'speed is set to ' + str(self.speed) +  '; rotate is set to ' +  str(self.rotate)
        print(msg)
        left_cmd  = self.speed - self.rotate
        right_cmd  = self.speed + self.rotate
        right_cmd = min(max(right_cmd,-255),255)
        left_cmd = min(max(left_cmd,-255),255)
        data = {"id": 1, "left":left_cmd, "right": right_cmd}
        requests.put(address + "agentTau/1", json=data)

    def keyup(self, event):
        if event.keysym in ["Up", "Down", "Left", "Right"]:
            print('Key arrow up')   
        elif event.keysym in ["q", "a", "w", "s", "e", "d", "r", "f", "t", "g", "y", "h"]:
            print('Key remain up') 
if __name__ == '__main__':
    control = ControlApp()
