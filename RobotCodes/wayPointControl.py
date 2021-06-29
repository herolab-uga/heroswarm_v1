#!/usr/bin/python
# -*- coding: utf-8 -*-
import socket
import pickle
import math
import numpy as np
import json
import os
import time

from numpy.lib.type_check import real
from SwarmRobot import SwarmRobot
from RobotPIDControl import PIDController

# Gets the robots host name

host = socket.gethostname()

# Opens the json file that stores the robots configuation data

# Global Variables initialization here
robot=None
with open('config/swarm_v1_config.JSON', 'r') as file:
    data = json.load(file)[host]

# Gets the right and lift duty cycles from json
right = data["right"]
left = data["left"]

# wayPoints=[(93,139),(95,109),(91,76),(92,36),(88,13),(80,24)]
wayPoints=[(92,36),(88,13),(80,24)]
wayPoint_delays=[0,0,0]
# wayPoint_delays=[0,0,0,0,0,0]

# Array that stores error values
# errors = np.zeros((PID.init if PID.init > PID.diff else PID.diff + 1))

def main():
    # Initializing Variables
    global robot
    global data
    query = 'o'
    # Instantiating the robot class drawing parameters from JSON
    robot = SwarmRobot(
        name=host,
        id=data['id'],
        linear_speed=data['linear_speed'],
        angular_speed=data['angular_speed'],
        pwm_pins=data["pwm_pins"]
        )

    #Initializing Connection with Localization System
    HOST = '192.168.1.78'
    PORT = 12346
    CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST, PORT))
    print('Connection Successfull!!')
    CLIENT_SOCKET.send(str(data['id']).encode('ascii'))
    # Initializing Robot
# This may not be required**
#    # Waits for sever to send odom info to robot
#     while robotOdo is None:
#         continue
    endPtPointer=0
    stpFlag=False
    while True:
        query = 'o'
        CLIENT_SOCKET.send(query.encode('ascii'))
        robotOdoP = CLIENT_SOCKET.recv(4096)
        robotOdo = pickle.loads(robotOdoP)
        (endPos,endPtPointer,stpFlag)=getEndPoint(robotOdo,endPtPointer,stpFlag)
        #     
        setMotion(robotOdo, endPos, stpFlag)
        stpFlag=checkDelay(endPtPointer,stpFlag)
        checkFinalRotation(endPtPointer,stpFlag,robotOdo)

def checkFinalRotation(endPtPtr,stpFlag,robotOdo):
    if stpFlag and endPtPtr==len(wayPoint_delays):
        print('Rotating to theta')
        RotateToTheta(robotOdo,90)



def checkDelay(endPtPtr,stpFlag):
    if stpFlag and endPtPtr<len(wayPoint_delays):
        time.sleep(wayPoint_delays[endPtPtr-1])
        stpFlag=False
    else:
        return stpFlag



def getEndPoint(robotodo,endPtPtr,stpFlag):
    if robotodo == None:
        endPos=None
    elif endPtPtr < len(wayPoints):
        endPtX=wayPoints[endPtPtr][0]
        endPtY=wayPoints[endPtPtr][1]
        endPtDelay=wayPoint_delays[endPtPtr]
        robotX = int(float(robotodo[0][0]))
        robotY = int(float(robotodo[0][1]))
        distToEndPt= math.sqrt((endPtX-robotX)**2+(endPtY-robotY)**2)
        if(distToEndPt<2):
            endPtPtr+=1
            stpFlag=True
            # if endPtPtr < len(wayPoints):
            #     stpFlag=False
            # else:
            
        endPos=[(endPtX,endPtY)]
        print('Pt'+str(endPtPtr)+'End Pt:'+str(endPos)+'Distance:'+str(distToEndPt)+'Stop Flag:'+str(stpFlag))
    else:
        endPos=None
    return(endPos,endPtPtr,stpFlag)
    
        
    








def setMotion(robotData, endPtData,stpFlag):
    global robot
    theta = None

    # Ensures there is robotData to evaluate if not return 
    # (Future implementation should include Kalman Filter here)
    if robotData == None or robotData[1] == None:
        return

    if endPtData != None and robotData != None:
        x = int(float(robotData[0][0]))
        y = int(float(robotData[0][1]))
        hx = int(float(robotData[0][2]))
        hy = int(float(robotData[0][3]))
        # Gets the ending position (Position of the target location)
        ex = int(float(endPtData[0][0]))
        ey = int(float(endPtData[0][1]))
        # trgDist = math.sqrt((x - ex) ** 2) + (y - ey) ** 2
        strtPt = np.array([x, y])
        endPt = np.array([ex, ey])
        headDir = np.array([hx, hy])

        # Gets the theta needed to correct heading and distance from target
        (theta, distance) = getThetaDistance(strtPt, endPt, headDir)

        if theta is None:
            robot.stop()
        else:
            MovOnTheta(theta, distance,stpFlag)
        
    else:
        # If at the tagert, update the position of the robot object
        if endPtData != None:
            robot.set_x(endPtData[0][0])
            robot.set_y(endPtData[0][1])
            robot.set_theta(robotData[1])
        robot.stop()

def RotateToTheta(robotData,setPtTheta):
    global robot
    x = int(float(robotData[0][0]))
    y = int(float(robotData[0][1]))
    hx = int(float(robotData[0][2]))
    hy = int(float(robotData[0][3]))
    heading_rob=[]
    thetaMargin=20
    thetaMargin1=setPtTheta+thetaMargin
    thetaMargin2=setPtTheta-thetaMargin

    heading_rob.append((float(hx) - float(x)))
    heading_rob.append((float(hy) - float(y)))

    # heading_rob_unit= np.divide(heading_rob , math.sqrt(heading_rob[0]**2+heading_rob[1]**2))


    robotTheta = np.degrees(np.arctan2(heading_rob[1],heading_rob[0])) # This has been verfied
    print(robotTheta)
    if robotTheta < thetaMargin1 and robotTheta > thetaMargin2:
        robot.stop()
        # print('Go Straight')
    elif robotTheta >=thetaMargin1:
        # print('Turning Right')
        robot.turn_right(59)
    elif robotTheta <=thetaMargin2:
        robot.turn_left(42)
        # print('Turning Left')




def MovOnTheta(theta, distance,stpFlag):
    global robot
    # stpFlag = False
    eStatus = True
    # Scales the margin angle with distance from the target (farther away less margin for error)
    #thetaMargin = 22.5 - distance / 232 * 22.5  # 116 - max distance possible between tag and robot
    thetaMargin=40
    thetaMargin1=thetaMargin/2
    thetaMargin2=-thetaMargin/2
    theta=180-theta
    # print(theta)
    if not stpFlag and eStatus:
        try:
            # print (theta)
            # If the robot heading is outside the target threshold (thetaMargin) turn the robot
            # This is a fuzzy controller
            if theta < thetaMargin1 and theta > thetaMargin2:
                robot.forward()
                # print('Go Straight')
            elif theta >=thetaMargin1:
                # print('Turning Right')
                robot.turn_right(60)
            elif theta <=thetaMargin2:
                robot.turn_left(41)
                # print('Turning Left')
        except Exception as e:
            print(e)
            robot.stop()
    else:
        robot.stop()

def getThetaDistance(startpoint, endpoint, heading):
    heading_rob = []
    # Gets the vector that describes the motion needed for robot to get to end point
    rob_end_vec = [float(endpoint[0]) - float(startpoint[0]),
                   float(endpoint[1]) - float(startpoint[1])]
    # Translates the heading of the robot to the be relative to robot center
    heading_rob.append((float(heading[0]) - float(startpoint[0])))
    heading_rob.append((float(heading[1]) - float(startpoint[1])))
    heading_rob_unit= np.divide(heading_rob , math.sqrt(heading_rob[0]**2+heading_rob[1]**2))
    rob_end_vec_unit= np.divide(rob_end_vec , math.sqrt(rob_end_vec[0]**2+rob_end_vec[1]**2))
    # Gets the rob_end_vec distance
    dif_dist=float(np.sqrt(float(rob_end_vec[0]) ** 2 + float(rob_end_vec[1]) ** 2))
    # If not a special case angle_end is described below
    angle_end = np.arctan2(rob_end_vec[1],rob_end_vec[0]) +math.pi# This has been verfied
    # print(np.degrees(angle_end))
    # Try to get the angle of the robot
    angle_robot = np.arctan2(heading_rob[1],heading_rob[0])+math.pi # This has been verfied
    # print(np.degrees(angle_robot))
    rl_angle=(math.degrees(np.arctan2(np.cross(heading_rob_unit,rob_end_vec_unit),np.dot(heading_rob_unit,rob_end_vec_unit))))+180
    return (rl_angle, dif_dist)


if __name__ == '__main__':
    main()
