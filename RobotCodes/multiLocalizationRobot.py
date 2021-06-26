import socket, pickle
import math
import numpy as np
import json
import os
import time

from numpy.lib.type_check import real
from SwarmRobot import SwarmRobot

host = socket.gethostname()
with open("config/swarm_v1_config.JSON","r") as file:
        data = json.load(file)[host]

class PID():
    init = 10
    diff = 5
    p_k = data["p"]
    i_k = data["i"]
    d_k = data["d"]
    max_speed = 15

    def get_angle(errors):
        p = PID.p_k * errors[0]
        i = PID.i_k * np.sum(errors)
        # d = PID.d_k * np.sum( -1 * np.diff(errors, n=PID.diff))
        d = 0
        return (p + i + d)
    
    def get_speed(PID_out):
        if np.abs(PID_out) > 180:
            PID_out = 180
        return np.abs((PID_out/180)) * PID.max_speed




robot = None
errors = np.zeros((PID.init if PID.init > PID.diff else PID.diff+1))
prev_theta = 0


def main():
    global robot
    HOST='192.168.1.78'
    PORT=12346
    global data
    CLIENT_SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST,PORT))
    print("1 Connected!!")
    CLIENT_SOCKET.send(str(data['id']).encode('ascii'))
    query='o'
    CLIENT_SOCKET.send(query.encode('ascii'))
    robotOdoP=CLIENT_SOCKET.recv(4096)
    robotOdo=pickle.loads(robotOdoP)
    while robotOdo is None:
        continue
    robot = SwarmRobot(
                        name=host,
                        id=data['id'],
                        linear_speed=data['linear_speed'],
                        angular_speed=data['angular_speed'],
                        x=robotOdo[0][0],
                        y=robotOdo[0][1],
                        theta=np.radians(robotOdo[1]))
    print(robot)
    while True:
        start = time.time()
        query='o'
        CLIENT_SOCKET.send(query.encode('ascii'))
        robotOdoP=CLIENT_SOCKET.recv(4096)
        robotOdo=pickle.loads(robotOdoP)
        query='e'
        CLIENT_SOCKET.send(query.encode('ascii'))
        endPosP=CLIENT_SOCKET.recv(4096)
        endPos=pickle.loads(endPosP)
        setMotion(robotOdo,endPos)
        # print(time.time() - start)

def setMotion(robotData,endPtData):
    global robot
    thetaMargin=10
    #print(data)
    stpFlag=False
    theta = None
    
    if robotData is None or robotData[1] is None:
        return

    # print("Robot Heading: " + str(robotData[1]))
    if not endPtData is None and not robotData is None:
            x=int(float(robotData[0][0]))
            y=int(float(robotData[0][1]))
            hx=int(float(robotData[0][2]))
            hy=int(float(robotData[0][3]))

            ex=int(float(endPtData[0][0]))
            ey=int(float(endPtData[0][1]))
            trgDist=(math.sqrt((x-ex)**2)+(y-ey)**2)
            strtPt=np.array([x,y])
            endPt=np.array([ex,ey])
            headDir=np.array([hx,hy])
            theta = getTheta(strtPt,endPt,headDir)
            # robot.set_theta(robotData[1])
            if not theta is None:
                MovOnTheta(theta)
            else:
                robot.stop()
    else:
        if not endPtData is None:
            robot.set_x(endPtData[0][0])
            robot.set_y(endPtData[0][1])
            robot.set_theta(robotData[1])
        robot.stop()


def MovOnTheta(theta):
    global errors
    global robot
    global prev_theta
    stpFlag=False
    eStatus=True
    thetaMargin=30
    error = -theta
    # print("Error: " + str(error))
    if not stpFlag and eStatus:
        try:
            print("Theta: " + str(theta))
            errors = np.insert(errors[:-1],0,error)
            # print(error)
            # print("PID: " + str(PID_out)) 
            if np.abs(theta) > thetaMargin:
                if theta <= 45 and theta > 0:
                    print("PID")
                    PID_out = PID.get_angle(errors)
                    robot.turn_right(PID.get_speed(PID_out))
                    # robot.turn_right(0)
                else:
                    if theta < 0:
                        print("Right")
                        robot.turn_right(0)
                    else:
                        print("Left")
                        robot.turn_left(4)
            else:
                print('Go Straight')
                robot.forward()
            # prev_theta = theta
        except Exception as e:
            print(e)
            robot.stop()
    else:
        #print('Stop')
        robot.stop()



def distance(vector):
    return float(np.sqrt(float(vector[0])**2 + float(vector[1])**2))

def getTheta(startpoint,endpoint,heading) -> float:
    heading_rob = []
    rob_end_vec = [float(endpoint[0])-float(startpoint[0]), float(endpoint[1])-float(startpoint[1])]
    heading_rob.append(float(heading[0])-float(startpoint[0]))
    heading_rob.append(float(heading[1])-float(startpoint[1]))
    # print(heading_rob)
    # print(dif)
    dif_dist = distance(rob_end_vec)

    if rob_end_vec[0] == 0:
        if rob_end_vec[1] < 0:
            rl_angle = np.deg2rad(90)
        else:
            rl_angle = np.deg2rad(-90)
    else:
        angle_end = np.arctan(rob_end_vec[1]/rob_end_vec[0])
        try:
            angle_robot = np.arctan(heading_rob[1]/heading_rob[0])
        except ZeroDivisionError:
            if heading[1] < startpoint[1]:
                angle_robot = np.deg2rad(-90)
            else:
                angle_robot = np.deg2rad(90)

        if angle_end > angle_robot:
            rl_angle = angle_end - angle_robot
        else:
            rl_angle = angle_robot - angle_end
    
    if (distance(heading - endpoint)) > dif_dist:
        rl_angle = math.pi

    return np.degrees(rl_angle)

if __name__ == '__main__':
    main()
