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

class Point3D():
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, _x, _y, _z):
        self.x = _x
        self.y = _y
        self.z = _z

class PositionEstimationStrategy(ABC):
    def __init__(self):
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


class KalmanEstimatePositionStrategy(PositionEstimationStrategy):
    def estimatePositionLogCallback(self, timestamp, data, logconf):
        self.position_estimate[0] = data['kalman.stateX']
        self.position_estimate[1] = data['kalman.stateY']
        self.position_estimate[2] = data['kalman.stateZ']
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
        self.position_estimate_event.set()

    def add_variables(self, logconf):
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')


# This function helps to see if the position estimate diverges
def wait_for_position_estimator(scf):
    print('\tWaiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            print("\t{} {} {}".format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)