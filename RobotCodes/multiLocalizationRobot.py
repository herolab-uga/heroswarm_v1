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
    print('Connection Successfull!!')
    CLIENT_SOCKET.send(str(data['id']).encode('ascii'))
    # Initializing Robot
# This may not be required**
#    # Waits for sever to send odom info to robot
#     while robotOdo is None:
#         continue


    while True:
        query = 'o'
        CLIENT_SOCKET.send(query.encode('ascii'))
        robotOdoP = CLIENT_SOCKET.recv(4096)
        robotOdo = pickle.loads(robotOdoP)
        query = 'e'
        CLIENT_SOCKET.send(query.encode('ascii'))
        endPosP = CLIENT_SOCKET.recv(4096)
        endPos = pickle.loads(endPosP)
        setMotion(robotOdo, endPos)


def setMotion(robotData, endPtData):
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
            MovOnTheta(theta, distance)
        
    else:
        # If at the tagert, update the position of the robot object
        if endPtData != None:
            robot.set_x(endPtData[0][0])
            robot.set_y(endPtData[0][1])
            robot.set_theta(robotData[1])
        robot.stop()


def MovOnTheta(theta, distance):
    global robot
    stpFlag = False
    eStatus = True
    # Scales the margin angle with distance from the target (farther away less margin for error)
    #thetaMargin = 22.5 - distance / 232 * 22.5  # 116 - max distance possible between tag and robot
    thetaMargin=30
    thetaMargin1=thetaMargin/2
    thetaMargin2=-thetaMargin/2
    # error = -theta
    if not stpFlag and eStatus:
        try:
            print (theta)
            # If the robot heading is outside the target threshold (thetaMargin) turn the robot
            if theta > thetaMargin1:
                robot.turn_right(46)
            elif theta<thetaMargin2:
                robot.turn_left(54)
            else:
                print('Go Straight')
                robot.forward()
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
    # Gets the rob_end_vec distance
    dif_dist=float(np.sqrt(float(rob_end_vec[0]) ** 2 + float(rob_end_vec[1]) ** 2))
    # If not a special case angle_end is described below
    angle_end = np.arctan2(rob_end_vec[1],rob_end_vec[0])
    # Try to get the angle of the robot
    angle_robot = np.arctan2(heading_rob[1],heading_rob[0])
    # Calculates the rl_angle for the robot
    if angle_end > angle_robot:
        rl_angle = angle_end - angle_robot
        endLeading = True
    else:
        rl_angle = angle_robot - angle_end
        endLeading = False
    # Correction made on angle due to some inconsistensies (Should be sorted in the future)
    # Returning the angle in degrees, the distance from the tag
    if np.degrees(rl_angle) >= 180:
                rl_angle = 360 - np.degrees(rl_angle)
    else:
        rl_angle = np.degrees(rl_angle)
    # Defining one side as negative and other as positive.
    rl_angle= -rl_angle if endLeading else rl_angle
    return (rl_angle, dif_dist)


if __name__ == '__main__':
    main()
