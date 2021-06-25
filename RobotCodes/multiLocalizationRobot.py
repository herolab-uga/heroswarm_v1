import socket, pickle
import math
import numpy as np
import json
import os
import time

from numpy.lib.type_check import real
from SwarmRobot import SwarmRobot

class PID():
    init = 5
    diff = 2
    p_k = 1
    i_k = 1
    d_k = 1
    max_speed = 22

    def get_angle(error):
        p = PID.p_k * error[0]
        i = PID.i_k * np.sum(error)
        d = PID.d_k * np.sum( -1 * np.diff(error, n=PID.diff))
        return (p + i + d)
    
    def get_speed(delta):
        return (delta/359) * PID.max_speed




robot = None
error = np.zeros((PID.init if PID.init > PID.diff else PID.diff+1))
prev_theta = 0


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
                        theta=np.radians(robotOdo[1]))
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
    global error
    global robot
    global prev_theta
    stpFlag=False
    eStatus=True
    thetaMargin=10
    delta_theta = theta - prev_theta
    if not stpFlag and eStatus:
        try:
            print("Delta Theta: " + str(theta))
            error = np.insert(error[:-1],0,delta_theta)
            print(error)
            angle = PID.get_angle(error)
            print("PID: " + str(angle))
            if np.abs(theta) > thetaMargin:
                robot.turn_right(PID.get_speed(angle))
            else:
                print('Go Straight')
                robot.forward()
            prev_theta = theta
        except Exception as e:
            print(e)
            robot.stop()
    else:
        #print('Stop')
        robot.stop()



def distance(vector):
    return np.sqrt(vector[0]**2 + vector[1]**2)

def getTheta(pt11,pt12,heading) -> float:
    dif = [pt12[0]-pt11[0], pt12[1]-pt11[1]]
    heading_length = distance(heading)
    dif_dist = distance(dif)
    angle = np.degrees(np.arccos(np.dot(heading,dif)/(heading_length*dif_dist)))
    return angle

if __name__ == '__main__':
    main()
