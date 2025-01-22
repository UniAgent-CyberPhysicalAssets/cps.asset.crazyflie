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
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

from cf_positioning import Point3D

DEFAULT_HEIGHT = 0.5
DEFAULT_VELOCITY = 0.3
BOX_LIMIT = 0.4

def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')

class CFOperationStrategy(ABC):
    _IS_DEBUG = False

    def __init__(self, scf: SyncCrazyflie, debug: bool = False):
        super().__init__()
        self._IS_DEBUG = debug
        if not scf:
            raise ValueError(f"scf: SyncCrazyflie must be set and cannot be None or empty.")
        
        self.mc = None
        self._scf = scf

    def isSetSCF(self):
        return self._scf != None
    
    def printDebug(self, msg: str):
        if self._IS_DEBUG:
            print(msg)

    @abstractmethod
    def activate_idle_simple(self):
        pass

    @abstractmethod    
    def shutdown(self):
        pass

    @abstractmethod
    def take_off_simple(self):
        pass

    @abstractmethod 
    def landing_simple(self):
        pass

    @abstractmethod
    def navigate_to_simple(self, targetPoint: Point3D):
        pass



class HlCommanderCFOperationImpl(CFOperationStrategy):
    
    def activate_idle_simple(self):
        self.printDebug("\tactivate_idle_simple")
        # x=0.0, y=0.0, z=0.0 # initial position
        #controller=CONTROLLER_MELLINGER #=2
        self.mc = PositionHlCommander(self._scf, 
                                      default_height=DEFAULT_HEIGHT, 
                                      default_velocity=DEFAULT_VELOCITY)
        # activate_mellinger_controller(cf=self._scf.cf)
        self.printDebug("\tI am idling now!")

    def shutdown(self):
        self.printDebug("shutdown")
        self.mc.stop()

    def take_off_simple(self):
        self.printDebug(f"\ttake off simple")
        self.mc.take_off(velocity=DEFAULT_VELOCITY+0.15) #take_off(self, height=None, velocity=0.2)
        #time.sleep(0.1)
        self.printDebug("Take off finished : I am hovering now")

    def landing_simple(self):
        self.printDebug("I am landing now!")
        self.mc.land()
        #time.sleep(0.1)
        self.printDebug("Technically, I am on the ground!")

    def navigate_to_simple(self, targetPoint: Point3D):
        self.printDebug(f"\tNavigate to: {targetPoint.x}, {targetPoint.y}, {targetPoint.z}")
        self.mc.go_to(targetPoint.x, targetPoint.y, targetPoint.z, velocity=0.3)
        self.printDebug("\tNavigation finished")

class DebugLoggingCFOperationImpl(CFOperationStrategy):
    timeSleep = 0.1
    def activate_idle_simple(self):
        self.printDebug("\tactivate_idle_simple")
        # x=0.0, y=0.0, z=0.0 # initial position
        #controller=CONTROLLER_MELLINGER #=2
        self.mc = PositionHlCommander(self._scf, default_height=DEFAULT_HEIGHT, default_velocity=DEFAULT_VELOCITY)
        activate_mellinger_controller(cf=self._scf.cf)
        time.sleep(self.timeSleep)
        self.printDebug("\tI am idling now!!")

    def shutdown(self):
        self.printDebug("shutdown")
        self.mc.stop()

    def take_off_simple(self):
        self.printDebug(f"\ttake off simple")
        time.sleep(self.timeSleep)
        self.printDebug("Take off finished : I am hovering now")

    def landing_simple(self):
        self.printDebug("I am landing now!")
        time.sleep(self.timeSleep)
        self.printDebug("Technically, I am on the ground!")

    def navigate_to_simple(self, targetPoint: Point3D):
        self.printDebug(f"\tNavigate to: {targetPoint.x}, {targetPoint.y}, {targetPoint.z}")
        time.sleep(self.timeSleep)
        self.printDebug("\tNavigation finished")
        
