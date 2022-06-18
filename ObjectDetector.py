import cv2
import numpy as np
from Frames_work import AStar_frame

'''
This function for scaling and displaying multi frames in a single 
frame for debugging and showing the result of each stage

'''
def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


'''
This function will take an image and apply some filter on it to improve the detection
'''
def preProcessing(img):
    imgBlur = cv2.GaussianBlur(img, (5, 5), 1)
    imgCanny = cv2.Canny(imgBlur, 220, 220)
    kernel = np.ones((5, 5))
    imgDial = cv2.dilate(imgCanny, kernel, iterations=2)
    imgThres = cv2.erode(imgDial, kernel, iterations=1)
    return imgThres

'''
This function will detect the object on the frame that was taken in the agreement and return 
a NumPy array with objects that have been detected
'''
def detect_objects(frame,Ar_corners):
    img = cv2.GaussianBlur(frame, (13, 13), 0)
    lowerb = np.array([0 ,0 ,0])
    upperb = np.array([150, 255, 255])
    maskb = cv2.inRange(img, lowerb, upperb)

    lowerg = np.array([0, 0, 0])
    upperg = np.array([255, 150, 255])
    maskg = cv2.inRange(img, lowerg, upperg)

    lowerr = np.array([0, 0, 0])
    upperr = np.array([255, 255, 150])
    maskr = cv2.inRange(img, lowerr, upperr)

    mask = maskb + maskg + maskr

    cutFrame = AStar_frame(mask)

    # Create a Mask with adaptive threshold
    imgThres = preProcessing(cutFrame)

    # show_adjust = stackImages(0.3, ([frame,imgThres],[cutFrame,mask]))
    # cv2.imshow('Show adjust', show_adjust)

    # Find contours
    contours, _ = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    center_aruco = []
    for aruco in Ar_corners:
        # center point of Aruco
        centerAruco = (int((aruco[0][0][0] + aruco[0][1][0] + aruco[0][2][0] + aruco[0][3][0]) / 4),
                       int((aruco[0][0][1] + aruco[0][1][1] + aruco[0][2][1] + aruco[0][3][1]) / 4))

        center_aruco.append(centerAruco)

    objects_contours = []
    for cnt in contours:
        flagX = True
        area = cv2.contourArea(cnt)
        if area > 500:
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            for center in range(len(center_aruco)):
                if (((center_aruco[center][0] - x) ** 2 + (center_aruco[center][1] - y) ** 2) ** 0.5) < 80:
                    flagX = False
                    break
            if (flagX):
                rect = (x, y), (w+150, h+150), angle
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.polylines(frame, [box], True, (255, 0, 0), 2)
                objects_contours.append(cnt)

    return objects_contours