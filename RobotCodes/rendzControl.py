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
        )

    #Initializing Connection with Localization System
    HOST = '192.168.1.78'
    PORT = 12346
    CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST, PORT))
    CLIENT_SOCKET.send(str(data['id']).encode('ascii'))
    print('Connection Successfull!!')
    
    # Initializing Robot
# This may not be required**
#    # Waits for sever to send odom info to robot
#     while robotOdo is None:
#         continue
    stpFlag=False

    while True:
        query = 'o'
        CLIENT_SOCKET.send(query.encode('ascii'))
        robotOdoP = CLIENT_SOCKET.recv(4096)
        robotOdo = pickle.loads(robotOdoP)
        # print(robotOdo)
        query = 'a'
        CLIENT_SOCKET.send(query.encode('ascii'))
        allOdoP = CLIENT_SOCKET.recv(4096)
        allOdo = pickle.loads(allOdoP)
        # print(allOdo)
        
        setMotion(robotOdo, allOdo)



def setMotion(robotData, allOdoData):
    global robot
    theta = None
    stpFlag=False
    # Ensures there is robotData to evaluate if not return 
    # (Future implementation should include Kalman Filter here)
    if robotData == None or robotData[1] == None:
        return

    if allOdoData != None and robotData != None:
        x = int(float(robotData[0][0]))
        y = int(float(robotData[0][1]))
        hx = int(float(robotData[0][2]))
        hy = int(float(robotData[0][3]))
        # Gets the ending position (Position of the target location)


        # Compute end 
        (ex,ey)=EndPointRend(allOdoData)
        stpFlag=checkEndPointDist(x,y,ex,ey)
        
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
        if allOdoData != None:
            (ex,ey)=EndPointRend(allOdoData)
            robot.set_x(ex)
            robot.set_y(ey)
            robot.set_theta(robotData[1])
        robot.stop()

def EndPointRend(allOdoData):
    xsum=0
    ysum=0
    tagCntr=0
    for tags in range(3,20):
        if tags !=robot.id:
            tag=str(tags)
            if(tag in allOdoData):
                thisOdo=allOdoData.get(tag)
                # print('odo'+str(thisOdo))
                xs=thisOdo[0][0]
                ys=thisOdo[0][1]
                # print('xs'+str(xs))
                xsum+=xs
                ysum+=ys
                tagCntr+=1
    print(tagCntr)
    if tagCntr!=0:
        meanX=xsum/tagCntr
        meanY=ysum/tagCntr
    else:
        meanX=None
        meanY=None
    # print('Means:'+str(meanX)+','+str(meanY))
    # print(allOdoData)
    return(meanX,meanY)

def checkEndPointDist(robotX,robotY,endPtX,endPtY):
    distanceThreshold=2
    if endPtX!=None and endPtY!=None and robotX!=None and robotY!=None:
        distanceReal=math.sqrt((robotX-endPtX)**2+(robotY-endPtY)**2)
        print('Distance:'+str(distanceReal))
        if distanceReal<=distanceThreshold:
            return True
        else:
            return False
    else:
        return True




def MovOnTheta(theta, distance,stpFlag):
    global robot
    eStatus = True
    # Scales the margin angle with distance from the target (farther away less margin for error)
    #thetaMargin = 22.5 - distance / 232 * 22.5  # 116 - max distance possible between tag and robot
    thetaMargin=40
    thetaMargin1=thetaMargin/2
    thetaMargin2=-thetaMargin/2

    pidStartWindow=10
    pidThetaMargin1=thetaMargin1+pidStartWindow
    pidThetaMargin2=thetaMargin2-pidStartWindow

    pidController1=PIDController(0.5,0)
    # pidController2=PIDController(0.5,0)

    # error = -theta
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
            
                
            # else:
            #     robot.stop()
            #     # print(pidController1.get_correction(theta))
                
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
    rl_angle=(math.degrees(np.arctan2(np.cross(heading_rob_unit,rob_end_vec_unit),np.dot(heading_rob_unit,rob_end_vec_unit))))+180

    return (rl_angle, dif_dist)


if __name__ == '__main__':
    main()
