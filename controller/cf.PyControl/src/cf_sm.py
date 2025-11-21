import asyncio
import logging
import sys
import time
from typing import Optional
from threading import Event
import threading

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

from cf_positioning import Point3D, PositionEstimationStrategy, KalmanEstimatePositionStrategy, StateEstimatePositionStrategy
from cf_drone_ops import CFOperationStrategy, HlCommanderCFOperationImpl, DebugLoggingCFOperationImpl

loggingPeriod_in_ms = 100 #ms

class StateMachineDrone(StateMachine):

    _uav_name = "uav1"

    uavOpStrategyImpl = None

    goalReached = False
    isFlying = False
    targetPointsQueue = []
    _IS_DEBUG = False

    def __init__(self, drone_id: str, debug: bool = False): 
        super().__init__()
        self._IS_DEBUG = debug
        if drone_id:
            self.set_uav_name(drone_id)
        self.current_transition = None

    @property
    def uav_name(self):
        return self._uav_name

    def set_uav_name(self, drone_id):
        if not drone_id:
            raise ValueError("drone_id must be set and cannot be None or empty")
        self._uav_name = drone_id

    def set_uavOpsImpl(self, uavOpsImpl):
        if not uavOpsImpl:
            raise ValueError(f"uavOpsImpl must be set and cannot be None or empty. uavOpsImpl must be of type {CFOperationStrategy.__qualname__}")
        self.uavOpStrategyImpl = uavOpsImpl

    def printDebug(self, msg: str):
        if self._IS_DEBUG:
            print(msg)

    # Define the OSGi lifecycle states
    installed = State('INSTALLED', initial=True)
    resolved = State('RESOLVED')
    starting = State('STARTING')
    active = State('ACTIVE')
    stopping = State('STOPPING')
    uninstalled = State('UNINSTALLED', final=True)

    # Define the UAV operation states
    idle = State('IDLE')
    hovering = State('HOVERING') #takeoff completed
    flying = State('FLYING')
    landed = State('LANDING')
    shutdown = State('SHUTDOWN')

    # Define the transitions for "OSGi" lifecycle (similar pattern here)
    install = installed.to(resolved, cond='dependencies_resolved')
    start = resolved.to(starting)
    initialize = starting.to(active)
    stop = active.to(stopping)
    stopped = stopping.to(resolved)
    uninstall = resolved.to(uninstalled)

    # Define the transitions for UAV operation
    activate_idle = active.to(idle)
    begin_takeoff = idle.to(hovering) #cond="reached_Height"
    begin_landing = hovering.to(landed)
    begin_nav_goal_sequence = hovering.to(flying)
    next_nav_goal = flying.to(flying, cond='goal_reached')
    keep_hovering = flying.to(hovering, cond='goal_reached')
    landing_completed = landed.to(idle, cond="is_not_flying")
    shutdown_command = idle.to(shutdown)
    begin_stopping = shutdown.to(stopping)

    # Conditional methods for transitions (if needed)
    def dependencies_resolved(self):
        # Implement logic to check if dependencies are resolved
        return True
    def goal_reached(self):
        # TODO check goal reached by accuracy.
        self.goalReached = True
        self.printDebug(f"<Condition/> reached for [{self.current_state}]: goal_reached={self.goalReached}")
        return self.goalReached
    def can_install(self):
        return True
    def is_not_flying(self):
        self.printDebug(f"<Condition> Checking for [{self.current_state}]: is_Flying={self.isFlying}")
        while(self.isFlying == True):
            time.sleep(loggingPeriod_in_ms/2/1000)
        self.printDebug(f"</Condition> reached for [{self.current_state}]: is_Flying={self.isFlying}")
        return True


    # Before/after_transition is for firing transitions only.
    def before_transition(self, event, state):
        # self.printDebug(f"BT: Before '{event}', on the '{state.id}' state.")
        self.current_transition = event
        self.writeSMGraph()
        
        self.writeSMGraph()
        return "before_transition_return"


    # This is for executing long-running UAV-actions
    def on_transition(self, event, state):
        self.printDebug(f"OT: On '{event}', on the '{state.id}' state.")
        self.writeSMGraph()
        
        # global lock
        # with lock:
        if event == "initialize":
            if(self.uavOpStrategyImpl == None or not self.uavOpStrategyImpl.isSetSCF()): 
                print("Not CF Operation Strategy selected or scf not initialized yet!")
                ISRUNNING = False
                raise Exception("Not CF Operation Strategy selected!")
        if event == "activate_idle":
            self.uavOpStrategyImpl.activate_idle_simple()
        if event == "begin_takeoff":
            self.uavOpStrategyImpl.take_off_simple()
        if event == "begin_landing":
            self.uavOpStrategyImpl.landing_simple()
        if event == "begin_nav_goal_sequence" or event == "next_nav_goal":
            navigate_to_simple(self)
        if event == "shutdown":
            self.uavOpStrategyImpl.shutdown()
            ISRUNNING = False
        
        self.writeSMGraph()
        return "on_transition_return"


    def on_exit_state(self, event, state):
        self.printDebug(f"OnExState: Exiting '{state.id}' state from '{event}' event.")
        self.writeSMGraph()
        
        self.writeSMGraph()
        return "on_exit_state"


    def on_enter_state(self, event, state):
        self.printDebug(f"OnEnState: Entering '{state.id}' state from '{event}' event.")
        self.writeSMGraph()
        if state == self.hovering:
            self.printDebug("\tHovering now")
            time.sleep(0.5)

        self.writeSMGraph()
        return "on_enter_state"

    # Before/after_transition is for firing transitions only.
    # Initiate Long-running action here via transitionFirings instead of on_transition (direct execution of UAV operation)
    ### After 'begin_nav_goal_sequence', on the 'flying' state: only when next_nav_goal is executed the 
    # condition goal_reached(self) on this transition is fired
    def after_transition(self, event, state):
        self.printDebug(f"AT: After '{event}', on the '{state.id}' state.")
        self.writeSMGraph()
        if (state == self.flying and (event == "next_nav_goal" or event == "begin_nav_goal_sequence")):
            if len(self.targetPointsQueue) != 0:
                self.printDebug(f"\tNext_Nav_Goal: There are still targets left: {len(self.targetPointsQueue)}")
                # TODO tryRepeat: when condition is false, tryMax
                self.next_nav_goal()
            else:
                self.printDebug("\tLet drone hover now, navgoals are empty")
                self.keep_hovering()
        #TODO: user-specific code
        if (state == self.landed): 
            self.printDebug("\tstate landed reached")
            self.landing_completed()
        self.writeSMGraph()
        return "after_transition"

    def writeSMGraph(self):
        asyncio.run(self.writePNG())

    async def writePNG(self):
        # Generate the state diagram
        smGraph = DotGraphMachine(self)
        smGraphPath = "./webview/img/" + self._uav_name + ".png"
        dot = smGraph()
        dot.write_png(smGraphPath)

    def get_current_state(self):
        return self.current_state.id
    
    def get_current_transition(self):
        return self.current_transition

# Track the flying state
def checkIfFlying(drone: StateMachineDrone):
    return drone.isFlying

# This method implements a "larger" action sequence in the state machine.
# It  implements navigation for drones based on a set of coordinates.
# It wraps calls to the atomic action "navigate_to_simple" of a drone
def navigate_to_simple(drone: StateMachineDrone):
    # global drone
    # global positionEstimator
    # if POSITIONING_SYSTEM == "LPS":
    if isinstance(drone.uavOpStrategyImpl.mc, PositionHlCommander):
        drone.printDebug(f"\tCurrent Position H1Commander: {drone.uavOpStrategyImpl.mc.get_position()}")
    # print(f"\tCurrent Position Estimate ({POSITION_ESTIMATE_FILTER}): {positionEstimator.position_estimate}")
    if len(drone.targetPointsQueue) != 0:
        targetPoint = drone.targetPointsQueue.pop(0)

        # Possibility to modify coordinates here, using other estimates
        # print(f"\tNavigate to: {position_estimate[0]}, {position_estimate[1]}, {targetPoint.z}")
        # mc.go_to(position_estimate[0], position_estimate[1], targetPoint.z, velocity=0.2)
        drone.uavOpStrategyImpl.navigate_to_simple(targetPoint=targetPoint)

        # if POSITIONING_SYSTEM == "LPS":
        if isinstance(drone.uavOpStrategyImpl.mc, PositionHlCommander):
            drone.printDebug(f"\tCurrent Position H1Commander (LPS): {drone.uavOpStrategyImpl.mc.get_position()}")
        # print(f"\tCurrent Position Estimate ({POSITION_ESTIMATE_FILTER}): {positionEstimator.position_estimate}")
        return
    drone.printDebug("\tAll nav goals finished! keep hovering now")
    drone.writeSMGraph()
    drone.keep_hovering()