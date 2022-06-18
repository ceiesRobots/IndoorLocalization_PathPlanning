import socket
import cv2
import numpy as np
from Frames_work import *
import Robot
from A_Star import pathfinding_astar as astar
import time
from multiprocessing import Process, Manager
import multiprocessing
from ObjectDetector import detect_objects
import threading
from future.moves import tkinter as tk
from future.moves.tkinter import font

demand_queue = list()
regions = {"1": (1025, 286), "2": (419, 497), "3": (1025, 717), "50": (1600, 497), "60": (1600, 497)}
charging = {}
reserved = [0, 0, 0, 0, 0]
region = 0


# Camera matrix and distortion coefficients for calibrating the frame
cameraMatrix = np.array([[1.54983576e+03, 0.00000000e+00, 1.03606279e+03],
                         [0.00000000e+00, 1.56256118e+03, 5.29394806e+02],
                         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist = np.array([[-0.39519925, 0.25529847, -0.00508874, -0.00172909, -0.11761777]])
size = (1920, 1080)
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, size, 1, size)
map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix,dist, np.identity(3),newCameraMatrix, size,cv2.CV_32FC2)
borders = np.array([]) # to cut the frame for exhibition

def get_input(demand_queue, regions):
    def getInput():
        global Label1
        top = tk.Tk()
        top.title("Choose a region")
        fontReg = font.Font(size=26, weight="bold")
        fontReset = font.Font(size=16, weight="bold")
        fontlbl = font.Font(size=12)
        Button1 = tk.Button(top, text="1", command=reg1, padx=40, pady=10, bg="#646464", fg="#FFFFFF", font=fontReg)
        Button2 = tk.Button(top, text="2", command=reg2, padx=40, pady=10, bg="#646464", fg="#FFFFFF", font=fontReg)
        Button3 = tk.Button(top, text="3", command=reg3, padx=40, pady=10, bg="#646464", fg="#FFFFFF", font=fontReg)
        Button4 = tk.Button(top, text=f"Charge/Uncharge Robot: 50", command=reg4, padx=45, pady=10, bg="#646464", fg="#FFFFFF", font=fontReset)
        Update = tk.Button(top, text=f"Charge/Uncharge Robot: 60", command=update, padx=45, pady=10, bg="#646464", fg="#FFFFFF",
                           font=fontReset)

        Label1 = tk.Label(top, pady=15, font=fontlbl)

        Button1.grid(row=0, column=0)
        Button2.grid(row=0, column=1)
        Button3.grid(row=0, column=2)
        Button4.grid(row=1, column=0, columnspan=3)
        Update.grid(row=2, column=0, columnspan=3)

        top.mainloop()

    def reg1():
        if not("1" in demand_queue) and reserved[0] == 0:
            demand_queue.append("1")
        Label1.config(text=demand_queue)
        print(demand_queue)

    def reg2():
        if not("2" in demand_queue) and reserved[1] == 0:
            demand_queue.append("2")
        Label1.config(text=demand_queue)
        print(demand_queue)

    def reg3():
        if not("3" in demand_queue) and reserved[2] == 0:
            demand_queue.append("3")
        Label1.config(text=demand_queue)
        print(demand_queue)

    def reg4():
        charging[50] = not(charging[50])
        Label1.config(text=demand_queue)
        print(demand_queue)

    def update():
        charging[60] = not(charging[60])
        Label1.config(text=demand_queue)
        print(demand_queue)

    getInput()

# Function finds the aruco marker's location
def findMarker(img, markerSize, totalMarkers=250, draw=True):
    grayImage = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)  # convert the image to grayscale
    dicType = getattr(ar, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')  # customizable function call
    arDict = ar.Dictionary_get(dicType)  # get the wanted dictionary from the available set
    arPara = ar.DetectorParameters_create()  # creates the needed aruco parameters to detect the markers
    corners, ids, rejected = ar.detectMarkers(grayImage, arDict, parameters=arPara)  # returns [[bounding box], [id], [rejected]]
    if draw:
        ar.drawDetectedMarkers(img, corners)  # draws the bounding box for each detected marker
    return [corners, ids]


# functions returns the nearest point
def nearest_robot(point, robot_ids, robots):
    free = False
    for i in robot_ids:
        if not(robots[i].hasTarget):
            free = True

    min = ((point[0] - robots[robot_ids[0]].Plocation[0]) ** 2 + (point[1] - robots[robot_ids[0]].Plocation[1]) ** 2)
    minIndex = 0
    if free:
        for i in range(1, len(robot_ids)):
            distance = ((point[0] - robots[robot_ids[i]].Plocation[0])**2 + (point[1] - robots[robot_ids[i]].Plocation[1] )**2)
            if distance < min and not robots[robot_ids[i]].hasTarget:
                min = distance
                minIndex = i
        return robot_ids[minIndex]
    else:
        return None


def astar_process(robot_id, shared_paths_state, shared_paths, frame, plocation, angle, point, scale):
    print(f'process for robot {robot_id} created')
    shared_paths_state[robot_id] = 1
    shape = (frame.shape[1] // scale, frame.shape[0] // scale)
    frame = cv2.resize(frame, shape)
    path = astar(frame, plocation, point, scale)

    shared_paths[robot_id] = path
    shared_paths_state[robot_id] = 0

# main program
def main():
    # Setting up communication configurations, and opening a socket to the robots
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('0.0.0.0', 8585))
    s.listen(0)

    process_manager = Manager()
    print("Starting...")
    vid = cv2.VideoCapture(1)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    # Aframe = np.ones((1080, 1920))

    move = True
    lock = multiprocessing.Lock()

    # Defining the robots, and a dictionary of robot objects
    robot_ids = [50, 60]
    names = ['car1.mshome.net', 'car2.mshome.net']
    robots = {}  # Ex. {'50': ROBOT_1, '60': ROBOT_2}
    global charging

    # Initializing robots and wireless connections
    n = len(robot_ids)
    i = 0
    while i < n:
        print(i)
        robots[robot_ids[i]] = Robot.Robot(robot_ids[i], names[i], cameraMatrix, s)
        charging[robot_ids[i]] = False
        current_name = socket.getfqdn(robots[robot_ids[i]].addr[0])
        if current_name != names[i]:
            robots[robot_ids[i]].disconnect()
            robots.pop(robot_ids[i])
            i -= 1
        i += 1
    print(robots)


    shared_paths = process_manager.dict()
    i = 1
    for id in robot_ids:
        shared_paths[id] = None

    shared_paths_state = process_manager.dict()
    i = 1
    for id in robot_ids:
        shared_paths_state[id] = 0 # zero for not calculating, 1 for calculating

    (threading.Thread(target=get_input, args=[demand_queue, regions])).start()

    have_borders = False  # a flag declares weather the orders are defined or not yet
    doAframe = False
    while True: # if there is a clientq
        prev_frame_time = time.time() #----------------- For calculating fps
        grabbed, frame = vid.read()  # graping the frame

        undist = cv2.remap(frame, map1, map2, cv2.INTER_CUBIC) # Undistorted frame
        corners, ids = findMarker(undist, 5)    # corners and ids of the arucos


        # updating robots information, (i.e location)
        for id in robot_ids:
            robots[id].updateInformation(undist, dist, corners, ids)

        # checking whether all borders are cropped or not yet
        if not have_borders:
            borders, have_borders, doAframe = getBorders(corners, ids, have_borders, doAframe)
            print((borders))
            print(have_borders)
            continue
        obstacles = detect_objects(undist, corners)

        if ids is not None:
            for id in robot_ids:
                if charging[id]:
                    robots[id].setTarget(regions[str(id)])
            if len(demand_queue) != 0:
                point = regions[demand_queue[0]]
                nearest = nearest_robot(point, robot_ids, robots)
                if nearest is not None:
                    print(f"robot:{nearest} is goint to point:{point}")
                    robots[nearest].setTarget(point)
                    reserved[int(demand_queue[0])-1] = nearest
                    demand_queue.remove(demand_queue[0])

        for id in robot_ids:
            if robots[id].reachedTarget:
                for res in range(len(reserved)):
                    if reserved[res] == id:
                        reserved[res] = 0
                        print("dealt with")
                if not (charging[id]):
                    robots[id].unsetTarget()



        for id in robot_ids:
            if not(robots[id].moving) and robots[id].hasTarget and not(robots[id].hasPath):
                if ids is not None and id in ids:

                    for robot in robots.values():
                        robot.client.send(robot.stop())
                    Aframe = AStar_frame(undist, True, obstacles,Ar_corners=corners)
                    if shared_paths_state[id] == 0:
                        lock.acquire()
                        move = False
                        robots[id].updateInformation(undist, dist, corners, ids)
                        locations, angles = robots[id].get_others(robots.values())

                        for o in range(0, len(locations)):
                            location = locations[0]
                            angle = angles[o]

                            rect = location, (350, 350), angle
                            box = cv2.boxPoints(rect)
                            box = np.int0(box)
                            cv2.fillPoly(Aframe, [box], (1))

                        robots[id].updateInformation(undist, dist, corners, ids)
                        print(robots[id].path)

                        p = Process(target=astar_process, args=(id, shared_paths_state, shared_paths, Aframe, robots[id].Plocation,
                                                                robots[id].angle, robots[id].target, 4))
                        p.start()
                        move = True
                        lock.release()
                else:
                    print('ids None? ', ids == None)

        for id in robot_ids:
            if (shared_paths[id] is not None) and not(robots[id].hasPath) and robots[id].hasTarget and not(robots[id].moving):
                print(f"Writing to robot: {id}")
                print(f"robot:{id}, going to point:{robots[id].target}, from point{robots[id].Plocation}")
                print(f"robot:{id} has the path {robots[id].path}")
                robots[id].setPath(shared_paths[id])
                shared_paths[id] = None
            else:
                shared_paths[id] = None

        # for x in shared_paths_state.keys(): print(shared_paths_state[x])
        if move:
            for id in robot_ids:
                if ids is not None and robots[id].id in ids and robots[id].hasTarget:
                    if robots[id].path is not None:
                        if not(robots[id].is_intersect(robots.values())):
                            robots[id].toPoint()



        scaledFrame = show_scaled_frame(undist)

        cv2.imshow('scaled image', scaledFrame)

        if cv2.waitKey(1) & 0xff == ord('q'):
            for robot in robots.values():
                robot.client.send(robot.stop())
            break
    s.close()
    vid.release()


if __name__ == '__main__':
    main()