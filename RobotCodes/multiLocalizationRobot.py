import socket, pickle
import math
import numpy as np
import json
import os
import time
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
                        host,
                        data['id'],
                        data['linear_speed'],
                        data['angular_speed'],
                        robotOdo[0],
                        robotOdo[1],
                        data['theta'])
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
        setMotion(robotOdo,endPos)


def setMotion(robotData,endPtData):
    thetaMargin=10
    #print(data)
    stpFlag=False
    theta = None

    if not endPtData is None and not robotData is None:
            x=int(float(robotData[0]))
            y=int(float(robotData[1]))
            hx=int(float(robotData[2]))
            hy=int(float(robotData[3]))

            ex=int(float(endPtData[0]))
            ey=int(float(endPtData[1]))
            trgDist=(math.sqrt((x-ex)**2)+(y-ey)**2)
            strtPt=np.array([x,y])
            endPt=np.array([ex,ey])
            headDir=np.array([hx,hy])
            theta =getTheta(strtPt,endPt,strtPt,headDir)
            theta=int(theta)

            MovOnTheta(theta)
    else:
        if not endPtData is None:
            robot.set_x(endPtData[0])
            robot.set_y(endPtData[1])
            robot.set_theta(theta)
        robot.stop()

def MovOnTheta (theta):
    stpFlag=False
    eStatus=True
    thetaMargin=25
    if not stpFlag and eStatus:
        try:
            if np.abs(theta-robot.get_theta()) > np.deg2rad(thetaMargin):
                print(theta)
                robot.turn(angle=theta)
            else:
                #print('Go Straight')
                robot.forward()
        except Exception as e:
         print(e)
    else:
        #print('Stop')
        robot.stop()





def getTheta(pt11,pt12) -> float:
    # print(pt11)
    vec1 = []
    vec1.append(pt12[0].astype(float)-pt11[0].astype(float))
    vec1.append(pt12[1].astype(float)-pt11[1].astype(float))

    vec12dt=math.atan(vec1[1]/vec1[0])
    print(vec12dt)
    return vec12dt

if __name__ == '__main__':
    main()
