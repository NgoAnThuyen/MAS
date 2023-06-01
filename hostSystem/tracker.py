"""
Code authored by Keegan Kelly
"""
import cv2
import numpy as np
import time
import math
import socket
import json
from threading import Thread
from webcamvideostream import WebcamVideoStream

import matplotlib.pyplot as plt

fig = plt.figure()
ax1 = fig.add_subplot(2,2,1)
ax2 = fig.add_subplot(2,2,2)
fig.show()

# time_ =0

class Tracker:
    UDP_IP = "192.168.10.124" # The IP that is printed in the serial monitor from the ESP32
    SHARED_UDP_PORT = 4210
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Internet  # UDP
    sock.connect((UDP_IP, SHARED_UDP_PORT))
    # importing the camera matrix and distortion coefficients
    # if the file path is working get rid of the folder or add a non relative path
    npfile = np.load("myCameraCalibration.npz")
    mtx = npfile["mtx"]
    dist = npfile["dist"]
    # declaring the dictionary that will store the corners of the markers at a specific id
    # marker 10 is the origin and then each agent is id 10 + agent number
    Corners = {10: tuple(), 11: tuple(), 12: tuple(), 13: tuple()}
    NUMMARKERS = 4
    # positions of each marker
    pos = np.zeros((NUMMARKERS, 3))
    pos[0] = [0, 0, np.pi/2]
    # bool used to only determine the position of the origin once to eliminate a bit of noise
    originFound = False
    # dictionary of aruco types
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }
    # constructor that takes the marker width and the aruco type

    def __init__(self, marker_width, aruco_type, fps=30, wideAngle=False):
        self.time_ = 0
        self.x_ = []
        self.y_ = []
        self.y_2 = []
        self.markerWidth = marker_width
        self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[aruco_type])
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.startTime = time.perf_counter()
        self.frameRate = fps
        self.wideAgnle = wideAngle
        if self.wideAgnle:
            npfile = np.load("myCameraCalibration.npz")
            self.mtx = npfile["mtx"]
            self.dist = npfile["dist"]
# Create figure for plotting
     


    def fixAngle(self, angle):
        # return an angle to -pi and pi
        while(angle > np.pi):
            angle -= 2*np.pi
        while(angle < -np.pi):
            angle += 2*np.pi
        return angle

    def find_markerPos(self, frame):
        # accepts a frame and locates markers and updates their positions and draws their position and info onto the frame
        # converts to gray scale and finds the aruco markers
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejectedImgPoints) = cv2.aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParams)
        # ads the corners to the dictionary if they are detected
        if len(corners) > 0:
            ids.flatten()
            for i in range(len(ids)):
                self.Corners[ids[i][0]] = corners[i]
        # only calculates position if the origin is found. Backup condition if the origin id has found markers because the origin will be marked as found inside the loop (second condition isn't checked if first condition is true)
        if self.originFound or len(self.Corners[10]) != 0:
            # locates the position of the origin marker only once to eliminate a bit of noise (if camera is not rigid and it moves this should be changed)
            if not self.originFound:
                # gets the rotation and translation vector of the origin marker
                self.originR, self.originT, markerpos = cv2.aruco.estimatePoseSingleMarkers(self.Corners[10], self.markerWidth, self.mtx, self.dist)
                # calculates rotation matrix from the rotation vector
                self.rodrigues = cv2.Rodrigues(self.originR[0][0])[0]
                self.originFound = True
            # calculates position of all other markers
            for i in range(1, self.NUMMARKERS):
                # checks if there is a marker found at the specific id
                if len(self.Corners[10+i]) != 0:
                    # finds marker position in the camera reference frame
                    rvec, tvec, markerpos = cv2.aruco.estimatePoseSingleMarkers(self.Corners[i+10], self.markerWidth, self.mtx, self.dist)
                    # finds the difference in position between the origin and the marker and rotates it to the origin reference frame
                    position = np.matmul(self.rodrigues, tvec[0][0]-self.originT[0][0])
                    # rotation matrix of the marker
                    Rod = cv2.Rodrigues(rvec[0][0])[0]
                    # multiply the rotation matrix of the marker with the rotation matrix of the origin and convert it back to a rotation vector. R_Z is the heading and the heading of the origin marker is added to the heading
                    heading = cv2.Rodrigues(np.matmul(self.rodrigues, Rod))[0][2] + np.pi/2*0
                    # updates the position of the marker
                    #self.pos[i] = [position[0], position[1], self.fixAngle(heading)]
                    self.pos[i][0] = position[0]
                    self.pos[i][1] = position[1]
                    self.pos[i][2] = self.fixAngle(heading)
                    # drawing axis on the markers
                    cv2.aruco.drawAxis(frame, self.mtx, self.dist, Rod, tvec, self.markerWidth)
        # draws the marker outlines, ids, and position onto the frame

        if self.originFound:
            # draws the axis on the origin marker
            cv2.aruco.drawAxis(frame, self.mtx, self.dist, self.rodrigues, self.originT[0][0], self.markerWidth * 2.5)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # calculating FPS and drawing it onto the frame
        self.endTime = time.perf_counter()
        dt = (self.endTime - self.startTime)
        
        self.startTime = self.endTime
        # preventing division by zero error
        if dt != 0:
            cv2.putText(frame, "FPS: " + format(1/dt, '.2f'), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # prints data
        LINE_UP = '\033[1A'
        LINE_CLEAR = '\x1b[2K'
        print(LINE_UP + LINE_CLEAR +"dt = "+ format(dt, '.4f') +"(" + format(self.pos[1][0], '.2f') + ", " + format(self.pos[1][1], '.2f') + ", " + format(self.pos[1][2], '.2f') + ")" + "(" + format(self.pos[2][0], '.2f') + ", " +
              format(self.pos[2][1], '.2f') + ", " + format(self.pos[2][2], '.2f') + ")" + "(" + format(self.pos[3][0], '.2f') + ", " + format(self.pos[3][1], '.2f') + ", " + format(self.pos[3][2], '.2f') + ")")
        
        return frame

    def startThreads(self):
        # starts threads for reading in new frames, displaying frames, processing frames, and sending data to the server
        self.Stop = False
        self.runGetFrame(frameRate=self.frameRate)
        t2 = Thread(target=self.runProcessFrame)
        t2.daemon = False
        t2.start()
        t3 = Thread(target=self.runShowFrame)
        t3.daemon = False
        t3.start()
        t1 = Thread(target=self.runPutThread)
        t1.daemon = False
        t1.start()
        t4 = Thread(target=self.runPlotThread)
        t4.daemon = False
        t4.start()

        plt.show()
        return self
    # ends the thread for put requests
    def stopThread(self):
        # stops all threads
        self.Stop = True
        self.vs.stop()
        self.vs.stream.release()
        cv2.destroyAllWindows()

    # sends a put request to the server with the locations of all markers at a poll rate of 20Hz


    def runPutThread(self):
        while not self.Stop:
            time.sleep(0.05)
            index  = 1
            m ={"id": "1","position": [self.pos[index][0],self.pos[index][1],self.pos[index][2]]}
            jsonObj = json.dumps(m)
            self.sock.send(jsonObj.encode())
    
    def runPlotThread(self):
        while not self.Stop:
            data = self.sock.recv(2048)
            pReq = json.loads(data)
            dataPos  = pReq["position"]
            global x, y, theta, theta_d
            x = dataPos[0]
            y = dataPos[1]
            theta =  dataPos[2]
            theta_d = pReq["desiredTheta"]
            de = pReq["ye"]
            error = math.atan2(math.sin(theta-theta_d),math.cos(theta-theta_d))
            # error = theta
            self.time_ = self.time_ + 0.1
            self.x_.append(self.time_)
            self.y_.append(error*(180.0/math.pi))
            self.y_2.append(de)
            self.x_ = self.x_[-100:]
            self.y_ = self.y_[-100:]
            self.y_2 = self.y_2[-100:]

            ax1.clear()
            ax1.plot(self.x_,self.y_, color='r')
            ax1.set_title("heading error (Deg)")
            ax2.clear()
            ax2.plot(self.x_,self.y_2, color='r')
            ax2.set_title("cross-track error (m)")
            fig.canvas.draw()
            # print("Heading error = " + str(error*180/math.pi))


    def runProcessFrame(self):
        # finds markers in the most recent frame in a loop
        while(True):
            if self.Stop:
                return
            if self.vs.grabbed:
                self.outFrame = self.find_markerPos(self.vs.frame)

    def runGetFrame(self, frameRate):
        if self.wideAgnle:
            Focus = 30
        else:
            Focus = 30
        # initializes the video stream
        self.vs = WebcamVideoStream(src=1, fps=frameRate, focus=Focus).start()
        self.vs.start()
        # sets an initial outframe to prevent crashing before the first frame is processed
        self.outFrame = self.vs.frame

    def runShowFrame(self):
        prevTime = time.time()
        frameDelta = 1/self.frameRate
        #output = cv2.VideoWriter("formation1.avi", cv2.VideoWriter_fourcc(*'MJPG'), 20, (1280, 720))
        while(True):
            # stops loop if thread is stopped
            if self.Stop:
                return
            # shows frame
            if self.vs.grabbed:
                prevTime = time.time()
                cv2.imshow('frame', self.outFrame)
                # output.write(self.outFrame)
            # stops all threads when q is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stopThread()
                break
            # reloads the origin position if r is pressed
            if cv2.waitKey(1) & 0xFF == ord('r'):
                self.originFound = False
            sleepTime = frameDelta - (time.time() - prevTime)
            time.sleep(sleepTime*(sleepTime > 0))
        return self


