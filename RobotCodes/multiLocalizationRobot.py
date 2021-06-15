import socket, pickle
import math
import numpy as np
import json
import os
from SwarmRobot import SwarmRobot

robot = None

def main():
    setup()
    HOST='192.168.1.78'
    PORT=12346
    CLIENT_SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST,PORT))
    print("1 Connected!!")
    CLIENT_SOCKET.send(str(robot.get_id()).encode('ascii'))
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


def setup():
    data = None
    host = os.environ.get("HOST")
    with open("config/swarm_v1_config.JSON","r",encoding='utf-8') as file:
        data = json.load(file)[host]
    robot = SwarmRobot(
                        host,
                        data['id'],
                        data['linear_speed'],
                        data['angular_speed'],
                        data['x_pos'],
                        data['y_pos'],
                        data['theta'])

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
        robot.set_x(endPtData[0])
        robot.set_y(endPtData[1])
        robot.set_theta(theta)
        robot.stop()

def MovOnTheta (theta):
     stpFlag=False

     eStatus=True
     thetaMargin=20        
     if not stpFlag and eStatus:     
          if np.abs(theta-robot.get_theta) < thetaMargin:
            robot.turn(angle=theta)
          else:
            #print('Go Straight')
            robot.forward()
     else:
          #print('Stop')
          robot.stop()





def getTheta(pt11,pt12,pt21,pt22):
    vec1=pt11-pt12
    vec2=pt22-pt21

    #vec12ds=math.degrees(math.asin2(np.cross(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2))))
    #vec12dc=math.degrees(math.asin2(np.cross(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2))))

    vec12dt=(math.degrees(math.atan2(np.cross(vec1,vec2),np.dot(vec1,vec2))))+180


    return vec12dt

if __name__ == '__main__':
    main()

