import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

'''
This function will get a coordinates of ArUco based on its position on the frame 
and return a new coordinates based on a position on the framework
'''
def to_cm(x, y):
    features = np.array([x, y, x * 2, y * 2])
    x_theta = np.array([1.03380635e+00, 6.19348985e-02, 1.39668938e-04, 1.99523240e-04])
    x_intercept = 164.9363105267435
    y_theta = np.array([-5.52365581e-02, 1.03991922e+00, -7.33148135e-05, -1.43090865e-05])
    y_intercept = 83.63579767123278
    out_x = (np.dot(features.T, x_theta)) + x_intercept + 5
    out_y = (np.dot(features.T, y_theta)) + y_intercept
    return [out_x, out_y]

font = cv2.FONT_HERSHEY_PLAIN

'''
Finding the distance and the angle difference between the Aruco and the destination ..
This function receives the Aruco info, its angle, and the destination point ..
However, it gives the distance and the angle between them!
'''
def navigationInfo(frame, id_to_find, Plocation, ids, angleAruco, destination):
    if ids is not None and id_to_find in ids:
            angleToDest = np.angle((destination[0] - Plocation[0]) + (Plocation[1] - destination[1]) * 1j, deg=bool)
            # angle difference (how much of rotating degrees needed to face the destination)
            angleDiff = (angleToDest - angleAruco)*-1
            if angleDiff > 180: angleDiff -= 360
            elif angleDiff < -180: angleDiff += 360
            # distance between robot and destination
            distance = (((Plocation[0] - destination[0]) ** 2 + (Plocation[1] - destination[1]) ** 2) ** 0.5) * (
                        18 / 81.89023)

            cv2.circle(frame, destination, 5, (0, 0, 255), -1)

            return distance, angleDiff
    else:
        # to avoid real time error
            distance, angleDiff = 0, 0
            return distance, angleDiff, 0