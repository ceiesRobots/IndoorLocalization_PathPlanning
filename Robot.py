import time
import numpy as np
import math
import cv2
import cv2.aruco as ar
import main
from Pose_Est import poseEstimation
from Navigation import navigationInfo
from Frames_work import *
import matplotlib.pyplot as plt
import pandas
from scipy.interpolate import interp1d

angle = []

class Robot:

    def __init__(self, id, name, cameraMatrix, socket):  # id = 50 for robot 1
        self.name = name
        self.id = id
        self.marker = self.setMarker()
        self.cameraMatrix = cameraMatrix
        self.speed = 0
        self.steer = 0
        self.moving = False
        self.path = None
        self.hasPath = False
        self.socket = socket
        self.calculating = False
        self.connect()
        self.hasTarget = False
        self.recalc = False
        self.count = 0
        self.prev_angleDiff = 0
        self.prevError = 0
        self.speed = 70
        self.steer = 60
        self.L = 120
        self.vic = 90
        if id == 60:
            self.speed = 60
            self.steer = 80
            self.L = 100
            self.vic = 90
        self.reachedTarget = False

    def connect(self):
        self.client, self.addr = self.socket.accept()
        # self.client.settimeout(10000)
        print(f'Robot() connected, address: {self.addr}')

    def disconnect(self):
        self.client.close()

    def updateInformation(self, undist, dist, corners, ids):
        self.frame_corners = corners
        self.frame_ids = ids
        self.x, self.y, self.angle = poseEstimation(undist, self.id, 18, self.cameraMatrix, dist, corners, ids)
        self.location = (self.x, self.y)
        if ids is not None and self.id in ids:
            index = np.where(ids == self.id)[0][0]
            self.Plocation = ((int(self.frame_corners[index][0][0][0] + self.frame_corners[index][0][1][0] +
                                   self.frame_corners[index][0][2][0] + self.frame_corners[index][0][3][
                                       0]) // 4),
                              int((self.frame_corners[index][0][0][1] + self.frame_corners[index][0][1][1] +
                                   self.frame_corners[index][0][2][1] + self.frame_corners[index][0][3][
                                       1]) // 4))
        else:
            self.Plocation = (0, 0)
        self.undist = undist

    def setMarker(self):
        newMarker = np.array([0])
        dicType = ar.Dictionary_get(ar.DICT_6X6_250)
        newMarker = ar.drawMarker(dictionary=dicType, id=self.id, sidePixels=200, img=newMarker)
        return newMarker

    def getMarker(self):
        return self.marker

    def getId(self):
        return self.id

    def getLocation(self):
        return self.location

    def getAngle(self):
        return self.angle

    def move(self, speed, steer):
        # if steer > 120 : steer = 120
        # if speed > 120 : speed = 120
        return np.array([speed, steer], dtype=np.byte)

    # Stopping the Robot
    def stop(self):
        return np.array([0, 0], dtype=np.byte)

    # Rotating the Robot
    def rotate(self, steer):
        return np.array([abs(steer) + 10, steer], dtype=np.byte)

    # PD Controller
    def PD(self, kp, kd, startPoint, endPoint):
        # startPoint = startPoint[::-1]
        endPoint = endPoint[::-1]
        L = self.L
        sign = self.angleDiff / abs(self.angleDiff)
        vectorP = np.subtract(endPoint, startPoint)
        # print('Vp: ', vectorP)
        vectorC = np.subtract(self.Plocation, startPoint)
        # print('Vc: ', vectorC)
        vectorProjection = vectorC.dot(vectorP) / (np.linalg.norm(vectorP) ** 2) * vectorP
        # print('Vprojection: ', vectorProjection)
        vectorE = np.subtract(vectorC, vectorProjection)
        if abs(self.angleDiff) > 160:
            vectorE = self.vic * vectorC
        elif abs(self.angleDiff >= 145):
            vectorE = (self.vic - 10) * vectorC

        e = ((np.linalg.norm(vectorE) * sign)/1.5 + L * math.sin(self.angleDiff * math.pi / 180)) * (18 / 82)
        de = e - self.prevError
        newAngle = int(kp * e - kd * de)
        self.prevError = e
        # print("new angle:", newAngle, "  angleDiff:", self.angleDiff, " error:", e, 'delta error:', de)
        return newAngle

    # Going to the destination with rotating
    def toPoint(self):
        global angle

        if len(self.path) > 0:
            point = self.path[0]

            if len(self.frame_corners) > 0:
                self.distance, self.angleDiff = navigationInfo(self.undist, self.id, self.Plocation,
                                                               self.frame_ids, self.angle,
                                                               point[::-1])  # endPoint)
            else:
                self.angleDiff = 0
                self.distance = 0

            maxAngle = 180
            maxSteer = self.steer
            newSpeed = self.speed
            sign = (self.angleDiff // abs(self.angleDiff))

            if self.start == None:
                self.start = self.Plocation
            endPoint = self.path[0]

            if (self.distance > 20):  # and len(self.path)>=2): #30 and len(self.path) <= 2) or (self.distance > 15 and len(self.path) > 2):
                pdAngle = self.PD(5, 0.3, np.asarray(self.start), np.asarray(endPoint))
                self.client.send(self.stop())
                self.client.send(self.stop())
                self.client.send(self.stop())
                self.client.send(self.stop())
                angle.append(pdAngle)
                newSteer = int((pdAngle / maxAngle) * (maxSteer))
                newSteer = (127 * sign) if abs(newSteer) >= 127 else newSteer
                # print("Steer:", newSteer)
                self.client.send(self.move(newSpeed, newSteer))
            else:
                self.client.send(self.stop())
                self.start = self.Plocation
                self.path.pop(0)

                if len(self.path) < 1:
                    self.reachedTarget = True

            self.prev_angleDiff = self.angleDiff
            return

    def setPath(self, path):
        # self.path = []
        self.moving = True
        self.hasPath = True
        self.path = path
        self.start = None

    def unsetPath(self):
        self.path = None
        self.moving = False
        self.hasPath = False
        self.recalc = True

    def is_intersect(self, robots):
        for other in robots:
            distance = math.sqrt(((self.Plocation[0] - other.Plocation[0]) ** 2) + (
                        (self.Plocation[1] - other.Plocation[1]) ** 2))
            if distance <= 550:
                if (other.id < self.id) and ((other.hasTarget)):
                    self.client.send(self.stop())
                    return True
                if (other.id > self.id) and (other.hasTarget) and not(self.recalc):
                    self.client.send(self.stop())
                    self.unsetPath()
                    print(f"path unsetted for robot {self.id}")
                    return True
        return False

    def setTarget(self, target):
        self.target = target
        self.reachedTarget = False
        self.hasTarget = True

    def unsetTarget(self):
        self.path = None
        self.hasPath = False
        self.moving = False
        self.recalc = False
        self.hasTarget = False
        self.target = None

    def get_others(self, robots):
        locations = []
        angles = []
        for other in robots:
            if other is not self:
                locations.append(other.Plocation)
                angles.append(other.getAngle())
        return locations, angles