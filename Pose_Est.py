import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math

# -- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# -- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# -- Changing position of Aruco in cm
def to_cm(x, y):
    features = np.array([x, y, x * 2, y * 2])
    x_theta = np.array([1.03380635e+00, 6.19348985e-02, 1.39668938e-04, 1.99523240e-04])
    x_intercept = 164.9363105267435
    y_theta = np.array([-5.52365581e-02, 1.03991922e+00, -7.33148135e-05, -1.43090865e-05])
    y_intercept = 83.63579767123278

    out_x = (np.dot(features.T, x_theta)) + x_intercept + 5
    out_y = (np.dot(features.T, y_theta)) + y_intercept
    return [out_x, out_y]

# -- Finding the position and angle of the Aruco
def poseEstimation(frame, id_to_find, marker_size, camera_matrix, camera_distortion, corners, ids):
    xAruco, yAruco, angleAruco = 0, 0, 0  # Initialize the values

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(R):
        assert (isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    # -- Find the aruco markers in the image
    if ids is not None and id_to_find in ids:
        index = np.where(ids == id_to_find)[0][0]

        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        # -- Unpack the output, get only the first
        rvec, tvec = ret[0][index, 0, :], ret[1][index, 0, :]

        # -- Put a reference frame over the wanted Marker
        aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

        # -- Obtain the rotation matrix tag->camera
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T

        # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
        _, _, angleAruco = rotationMatrixToEulerAngles(R_flip * R_tc)
        angleAruco = math.degrees(angleAruco) + 90
        if angleAruco > 180: angleAruco -= 360
        xAruco, yAruco = tvec[0], tvec[1]

        # -- Print the tag position in camera frame
        # str_position = "MARKER Position xAruco=%4.0f  yAruco=%4.0f" % (xAruco, yAruco)
        # cv2.putText(frame, str_position, (629, 860), font, 1.8, (0, 0, 0), 2, cv2.LINE_AA)
    # if xAruco < 1:
    return xAruco, yAruco, angleAruco