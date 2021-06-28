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

# Gets the robots host name

host = socket.gethostname()

# Opens the json file that stores the robots configuation data

with open('config/swarm_v1_config.JSON', 'r') as file:
    data = json.load(file)[host]


# PID controller class

class PID:

    init = 10
    diff = 5

    # Weights are stored in json file

    p_k = data['p']
    i_k = data['i']
    d_k = data['d']
    max_speed = 15

    # Computes the PID value usind the errors array

    def get_angle(errors):
        p = PID.p_k * errors[0]
        i = PID.i_k * np.sum(errors)

        # d = PID.d_k * np.sum( -1 * np.diff(errors, n=PID.diff))

        d = 0
        return p + i + d

    # Normalizes the PID value and uses to detirmine angular speed

    def get_speed(PID_out):
        if np.abs(PID_out) > 180:
            PID_out = 180
        return np.abs(PID_out / 180) * PID.max_speed


# Robot Object

robot = None

# Array that stores error values

errors = np.zeros((PID.init if PID.init > PID.diff else PID.diff + 1))


def main():
    global robot
    HOST = '192.168.1.78'
    PORT = 12346
    global data
    CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST, PORT))
    print('1 Connected!!')
    CLIENT_SOCKET.send(str(data['id']).encode('ascii'))
    query = 'o'
    CLIENT_SOCKET.send(query.encode('ascii'))
    robotOdoP = CLIENT_SOCKET.recv(4096)
    robotOdo = pickle.loads(robotOdoP)

    # Waits for sever to send odom info to robot

    while robotOdo is None:
        continue

    # Creates robot object using the config data

    robot = SwarmRobot(
        name=host,
        id=data['id'],
        linear_speed=data['linear_speed'],
        angular_speed=data['angular_speed'],
        x=robotOdo[0][0],
        y=robotOdo[0][1],
        theta=np.radians(robotOdo[1]),
        )
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
    thetaMargin = 10

    # print(data)

    stpFlag = False
    theta = None

    # Ensures there is robotData to evaluate if not return

    if robotData is None or robotData[1] is None:
        return

    if not endPtData is None and not robotData is None:

            # Gets the center (x,y) and the heading (hx,hy) of the robot

        x = int(float(robotData[0][0]))
        y = int(float(robotData[0][1]))
        hx = int(float(robotData[0][2]))
        hy = int(float(robotData[0][3]))

            # Gets the ending position (Position of the target location)

        ex = int(float(endPtData[0][0]))
        ey = int(float(endPtData[0][1]))
        trgDist = math.sqrt((x - ex) ** 2) + (y - ey) ** 2
        strtPt = np.array([x, y])
        endPt = np.array([ex, ey])
        headDir = np.array([hx, hy])

            # Gets the theta needed to correct heading and distance from target

        (theta, distance) = getTheta(strtPt, endPt, headDir)

            # robot.set_theta(robotData[1])

        if not theta is None:
            MovOnTheta(theta, distance)
        else:
            robot.stop()
    else:

        # If at the tagert, update the position of the robot object

        if not endPtData is None:
            robot.set_x(endPtData[0][0])
            robot.set_y(endPtData[0][1])
            robot.set_theta(robotData[1])
        robot.stop()


def MovOnTheta(theta, distance):
    global errors
    global robot
    global prev_theta
    stpFlag = False
    eStatus = True

    # Scales the margin angle with distance from the target (farther away less margin for error)

    thetaMargin = 22.5 - distance / 232 * 22.5  # 116 - max distance possible between tag and robot
    error = -theta
    if not stpFlag and eStatus:
        try:

            # print("Theta: " + str(theta))
            # Addes the current error to the array

            errors = np.insert(errors[:-1], 0, error)

            # print(error)
            # print("PID: " + str(PID_out))
            # If the robot heading is outside the target threshold (thetaMargin) turn the robot

            if np.abs(theta) > thetaMargin:
                print(theta)

                # If theta with in +/- 90 degrees of target heading use PID

                if theta <= 90 and theta > thetaMargin:
                    print('PID')
                    # PID_out = PID.get_angle(errors)
                    # robot.turn_right(PID.get_speed(PID_out))
                else:
                    # If theta outside +/- 90 degrees and less than zero turn right
                    # We have the most control over clockwise turns

                    if theta > 0:
                        print('Right')
                        # robot.turn_right(0)
                    else:
                        print('Left')
                        # robot.turn_left(3)
            else:
                print('Go Straight')
                # robot.forward()
        except Exception as e:

            # prev_theta = theta

            print(e)
            robot.stop()
    else:

        # print('Stop')

        robot.stop()


def distance(vector):
    return float(np.sqrt(float(vector[0]) ** 2 + float(vector[1]) ** 2))


def getTheta(startpoint, endpoint, heading):
    heading_rob = []

    # Gets the vector that describes the motion needed for robot to get to end point

    rob_end_vec = [float(endpoint[0]) - float(startpoint[0]),
                   float(endpoint[1]) - float(startpoint[1])]

    # Translates the heading of the robot to the be relative to robot center

    heading_rob.append(float(heading[0]) - float(startpoint[0]))
    heading_rob.append(float(heading[1]) - float(startpoint[1]))

    # Gets the rob_end_vec distance

    dif_dist = distance(rob_end_vec)

    # If not a special case angle_end is described below

    angle_end = np.arctan2(rob_end_vec[1],rob_end_vec[0])

    # Try to get the angle of the robot
    
    angle_robot = np.arctan2(heading_rob[1],heading_rob[0])

    # Calculates the rl_angle for the robot

    if angle_end > angle_robot:
        rl_angle = angle_end - angle_robot
    else:
        rl_angle = angle_robot - angle_end

    if rl_angle > math.pi and startpoint[1] > endpoint[1]:
        rl_angle += -2*math.pi


    # Return the angle in degrees and the distance from the tag

    return (np.degrees(rl_angle), dif_dist)


if __name__ == '__main__':
    main()
