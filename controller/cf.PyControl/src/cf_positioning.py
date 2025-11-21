import logging
import sys
import time
from threading import Event
import threading
from abc import ABC, abstractmethod

from flask import Flask, jsonify, request
from werkzeug.routing import BaseConverter, ValidationError
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine



import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

class Point3D():
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, _x, _y, _z):
        self.x = _x
        self.y = _y
        self.z = _z

class PositionEstimationStrategy(ABC):
    def __init__(self, log_values):
        self.log_values = log_values
        # Event to signal when the position estimate is available
        self.position_estimate_event = threading.Event() 
        # Vector to store the last position estimate
        self.position_estimate = [0, 0, 0]

    @abstractmethod
    def estimatePositionLogCallback(self, timestamp, data, logconf):
        pass

    @abstractmethod
    def add_variables(self, logconf):
        pass

    def get_log_values(self):
        return self.log_values

class KalmanEstimatePositionStrategy(PositionEstimationStrategy):
    def estimatePositionLogCallback(self, timestamp, data, logconf):
        self.position_estimate[0] = data['kalman.stateX']
        self.position_estimate[1] = data['kalman.stateY']
        self.position_estimate[2] = data['kalman.stateZ']
        self.log_values['x'] = data['kalman.stateX']
        self.log_values['y'] = data['kalman.stateY']
        self.log_values['z'] = data['kalman.stateZ']
        self.position_estimate_event.set()

    def add_variables(self, logconf):
        logconf.add_variable('kalman.stateX', 'float')
        logconf.add_variable('kalman.stateY', 'float')
        logconf.add_variable('kalman.stateZ', 'float')

class StateEstimatePositionStrategy(PositionEstimationStrategy):
    def estimatePositionLogCallback(self, timestamp, data, logconf):
        self.position_estimate[0] = data['stateEstimate.x']
        self.position_estimate[1] = data['stateEstimate.y']
        self.position_estimate[2] = data['stateEstimate.z']
        self.log_values['x'] = data['stateEstimate.x']
        self.log_values['y'] = data['stateEstimate.y']
        self.log_values['z'] = data['stateEstimate.z']
        self.position_estimate_event.set()

    def add_variables(self, logconf):
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')

# Delegate to cflib
def reset_estimator0(scf, console):
    # console.print('\tWaiting for estimator to find position...')
    reset_estimator(scf.cf)
    # console.print('Waiting for estimator to find position...success!')