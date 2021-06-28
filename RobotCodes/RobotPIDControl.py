import socket
import pickle
import math
import numpy as np
import json
import os
import time
from numpy.lib.type_check import real
from SwarmRobot import SwarmRobot

class PIDController:
    def __init__(self,parameters):
    # This PID is for alignment with the endpoint
        init = 10 # Integral window
        diff = 5  # Differentiator

        p_k = parameters['p']
        i_k = parameters['i']
        d_k = parameters['d']
        max_speed = 15

        # Computes the PID value usind the errors array
        def get_angle(errors):
            p_control = PID.p_k * errors[0]
            i_control = PID.i_k * np.sum(errors)
            # d = PID.d_k * np.sum( -1 * np.diff(errors, n=PID.diff))
            d_control = 0
            correctionSignal=p_control+i_control+d_control
            return correctionSignal

        # Normalizes the PID value and uses to detirmine angular speed

        def get_speed(PID_out):
            if np.abs(PID_out) > 180:
                PID_out = 180
            return np.abs(PID_out / 180) * PID.max_speed
