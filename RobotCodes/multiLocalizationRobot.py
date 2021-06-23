import socket, pickle
import math
import numpy as np
import json
import os
import time

from numpy.lib.type_check import real
from SwarmRobot import SwarmRobot

robot = None

def main():
    global robot
    HOST='192.168.1.78'
    PORT=12346
    host = socket.gethostname()
    data = None
    with open("config/swarm_v1_config.JSON","r") as file:
        data = json.load(file)[host]

    CLIENT_SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST,PORT))
    print("1 Connected!!")
    time.sleep(10)
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
                        theta=np.deg2rad(robotOdo[1]))
    print(robot)
    while True:
        query='o'
        CLIENT_SOCKET.send(query.encode('ascii'))
        robotOdoP=CLIENT_SOCKET.recv(4096)
        robotOdo=pickle.loads(robotOdoP)
        query='e'
        CLIENT_SOCKET.send(query.encode('ascii'))
        endPosP=CLIENT_SOCKET.recv(4096)
        endPos=pickle.loads(endPosP)
        # print("Robot")
        # print(robotOdo)
        # print("End")
        # print(endPos)
        setMotion(robotOdo,endPos)


def setMotion(robotData,endPtData):
    global robot
    thetaMargin=10
    #print(data)
    stpFlag=False
    theta = None
    
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
            theta = getTheta(strtPt,endPt)
            robot.set_theta(np.deg2rad(robotData[1]))
            if not theta is None:
                MovOnTheta(theta)
            else:
                robot.stop()
    else:
        if not endPtData is None:
            robot.set_x(endPtData[0][0])
            robot.set_y(endPtData[0][1])
            robot.set_theta(np.deg2rad(robotData[1]))
        robot.stop()


def MovOnTheta(theta):
    global robot
    stpFlag=False
    eStatus=True
    thetaMargin=20
    if not stpFlag and eStatus:
        try:
            print("Delta: " + str(theta-robot.get_theta()))
            # print(np.abs(theta-robot.get_theta()) > np.deg2rad(thetaMargin))
            if np.abs(theta-robot.get_theta()) > np.deg2rad(thetaMargin):
                # print(np.abs(theta-robot.get_theta()) > np.deg2rad(thetaMargin))
                robot.turn(angle=theta)
            else:
                print('Go Straight')
                robot.forward()
        except Exception as e:
            print(e)
            robot.stop()
    else:
        #print('Stop')
        robot.stop()





def getTheta(pt11,pt12) -> float:
    # print(pt11)
    vec1 = []
    vec1.append(pt12[0].astype(float)-pt11[0].astype(float))
    vec1.append(pt12[1].astype(float)-pt11[1].astype(float))

    vec12dt=np.arctan2(vec1[1],vec1[0])
    print("Theta: " + str(vec12dt))
    return vec12dt

if __name__ == '__main__':
    main()
