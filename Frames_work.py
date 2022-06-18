import numpy as np
import cv2
import cv2.aruco as ar


''' Initial values for the Corners of frame worke '''
borders = [[0, 0], [1292, 0], [0, 646], [1292, 646]]
cropCorners1 = [10, 20, 30, 40]


''' 
This function will change the value of borders to the position of the first 
corner (top-left) of each ArUco ID in cropCorners1 array then return the 
borders as numpy array

@param  corners: Numpy array of all corners that detect in the frame
        ids: Numpy array of all ids that detect in the frame
        have_borders: a flag that indicates to detect all ids in cropCorners
        doAframe: 

@return borders: Numpy array of the position of the top-left corner
        have_borders: a flag that indicates to detect all ids in cropCorners
        doAframe: 
'''
def getBorders(corners, ids, have_borders, doAframe):
    global borders
    global cropCorners1

    # find Corners And Ids
    if ids is not None:
        for id in cropCorners1:
            if id == 0: continue  # To bypass unwanted corners to update
            for index in range(len(ids)):
                if int(ids[index]) == id:
                    # remove noise
                    if abs(borders[cropCorners1.index(id)][0] - int(corners[index][0][0][0])) > 10 or abs(
                            borders[cropCorners1.index(id)][1] - int(corners[index][0][0][1])) > 10:
                            borders[cropCorners1.index(id)] = [corners[index][0][0][0], corners[index][0][0][1]]
                            cropCorners1[cropCorners1.index(id)] = 0
    if cropCorners1 == [0, 0, 0, 0]:
        have_borders = True
        doAframe = True
    return np.array(borders), have_borders, doAframe


'''
This function will crop a frame based on the coordinates in borders array

@param: frame: The frame that needs to be cropped

@return: perspectiveImg: a new cropped frame
'''
def show_scaled_frame(frame):
    global borders
    global cropCorners1
    corner = borders

    XExpand, YExpand = 20, 10
    Xpoints = [corner[0][0] - XExpand, corner[1][0] + XExpand, corner[2][0] - XExpand, corner[3][0] + XExpand]
    Ypoints = [corner[0][1] - YExpand, corner[1][1] - YExpand, corner[2][1] + YExpand, corner[3][1] + YExpand]
    points1 = np.float32([[Xpoints[0], Ypoints[0]], [Xpoints[1], Ypoints[1]], [Xpoints[2], Ypoints[2]], [Xpoints[3], Ypoints[3]]])
    points2 = np.float32([[0, 0], [(1292), 0], [0, (646)], [(1292), (646)]])

    matrix = cv2.getPerspectiveTransform(points1, points2)
    perspectiveImg = cv2.warpPerspective(frame, matrix, ((1292), (646)))

    return perspectiveImg

'''
This function will be used for two purposes, first one is for object detection and 
it will return the frame after putting a black mask around the framework based on 
the coordinates in borders array.
The second purpose is to prepare the frame for A_Star algorithm and it will return 
a frame with zeros inside the framework and ones outside the framework. Moreover, 
it will put an ones mask on obstacles.

@param: frame: The main frame.
        A_star: A flag that indicates the usage of this function.
        objects_contours: A numpy array of objects.
        Ar_corners: A numpy array of ArUco corners.

@return: mask: a new frame with the mask

'''
def AStar_frame(frame,A_star=False,objects_contours=[],Ar_corners=[]):
    global borders
    global cropCorners1
    corner = borders

    borderX = (int(corner[2][0])+100,int(corner[1][0]))
    borderY = [0,0]

    borderY[0] = int(corner[0][1]) if corner[0][1] > corner[1][1] else int(corner[1][1])
    borderY[1] = int(corner[2][1]) if corner[2][1] < corner[3][1] else int(corner[3][1])
    if (corner[0][1] - corner[2][1]) != 0 or (corner[1][1] - corner[3][1]) != 0:
        slop1 = (corner[0][0] - corner[2][0]) / (corner[0][1] - corner[2][1])
        slop2 = (corner[1][0] - corner[3][0]) / (corner[1][1] - corner[3][1])
    else:
        slop1, slop2 = 0, 0



    if A_star:
        type = None
        frame = np.zeros(frame.shape[:2], dtype=type)
    else:
        type = np.ubyte

    mask = np.ones(frame.shape, dtype=type)

    for i in range(borderY[0],borderY[1]):
        start = slop1 * (i-borderY[0]) + borderX[0]
        end = slop2 * (i-borderY[1]) + borderX[1]
        mask[i, int(start):int(end)] = frame[i, int(start):int(end)]

    center_aruco = []
    if Ar_corners is not None:
        for aruco in Ar_corners:
            # center point of Aruco
            centerAruco = (int((aruco[0][0][0] + aruco[0][1][0] + aruco[0][2][0] + aruco[0][3][0]) / 4),
                           int((aruco[0][0][1] + aruco[0][1][1] + aruco[0][2][1] + aruco[0][3][1]) / 4))

            center_aruco.append(centerAruco)

    for cnt in objects_contours:
        rect = cv2.minAreaRect(cnt)
        (x, y), (w, h), angle = rect

        rect = (x, y), (w + 150, h + 150), angle

        # Display rectangle
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.fillPoly(mask, [box], (1))

    return mask