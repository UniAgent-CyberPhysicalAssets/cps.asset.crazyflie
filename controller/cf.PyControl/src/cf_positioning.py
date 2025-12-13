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

# Additional imports for ROS-based position estimation
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseArray
except Exception:
    # rclpy might not be available in environments where ds_crazyflie is not used.
    rclpy = None
    Node = object
    PoseArray = object

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
        self.position_estimate_event = threading.Event()
        self.position_estimate = [0, 0, 0]

    def requires_position_before_initialize(self) -> bool:
        return False

    @abstractmethod
    def estimatePositionLogCallback(self, timestamp, data, logconf):
        pass

    @abstractmethod
    def add_variables(self, logconf):
        pass

    def get_log_values(self):
        return self.log_values

    def is_ready(self) -> bool:
        return self.position_estimate_event.is_set()

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

# ROS2 PoseArray based position estimation strategy
class RosPoseArrayPositionStrategy(PositionEstimationStrategy, Node):
    """
    Position estimation strategy that subscribes to a ROS2 PoseArray topic and extracts the first pose as the current
    Crazyflie position. It updates the shared log_values dictionary accordingly.
    """
    def __init__(self, log_values, cf_id: int = 0, topic: str = '/cf_positions_poses'):
        PositionEstimationStrategy.__init__(self, log_values)
        if rclpy is None:
            raise ImportError("rclpy is not available. RosPoseArrayPositionStrategy requires ROS2.")
        Node.__init__(self, f'ds_position_estimator_cf{cf_id}')
        self.cf_id = cf_id

        self.subscription = self.create_subscription(
            PoseArray,
            topic,
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return
        pose = msg.poses[self.cf_id]

        self.position_estimate[0] = pose.position.x
        self.position_estimate[1] = pose.position.y
        self.position_estimate[2] = pose.position.z
        self.log_values['x'] = pose.position.x
        self.log_values['y'] = pose.position.y
        self.log_values['z'] = pose.position.z

        self.position_estimate_event.set()

    # Unused in ROS-based strategy
    def estimatePositionLogCallback(self, timestamp, data, logconf):
        pass

    # Unused in ROS-based strategy
    def add_variables(self, logconf):
        pass

# Delegate to cflib
def reset_estimator0(scf, console):
    # console.print('\tWaiting for estimator to find position...')
    reset_estimator(scf.cf)
    # console.print('Waiting for estimator to find position...success!')