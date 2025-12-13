import asyncio
import time
from cflib.positioning.position_hl_commander import PositionHlCommander
from flask import Flask, jsonify, request
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine
from werkzeug.routing import BaseConverter, ValidationError
from cf_drone_ops import CFOperationStrategy


class StateMachineDrone(StateMachine):
    _IS_DEBUG = False
    loggingPeriod_in_ms = 100  # ms

    _uav_name = "uav1"
    uavOpStrategyImpl = None
    goalReached = False
    isFlying = False
    targetPointsQueue = []

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
            raise ValueError(
                f"uavOpsImpl must be set and cannot be None or empty. uavOpsImpl must be of type {CFOperationStrategy.__qualname__}")
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
    hovering = State('HOVERING')  # takeoff completed
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
    begin_takeoff = idle.to(hovering)  # cond="reached_Height"
    begin_landing = hovering.to(landed)
    begin_nav_goal_sequence = hovering.to(flying)
    next_nav_goal = flying.to(flying, cond='goal_reached')
    keep_hovering = flying.to(hovering, cond='goal_reached')
    landing_completed = landed.to(idle, cond="is_not_flying")
    shutdown_command = idle.to(shutdown)
    begin_stopping = shutdown.to(stopping)

    # Conditional methods for transitions (if needed)
    def dependencies_resolved(self):
        return True

    # Add custom logic here
    def goal_reached(self):
        self.goalReached = True
        self.printDebug(f"<Condition/> reached for [{self.current_state}]: goal_reached={self.goalReached}")
        return self.goalReached

    def can_install(self):
        return True

    def is_not_flying(self):
        self.printDebug(f"<Condition> Checking for [{self.current_state}]: is_Flying={self.isFlying}")
        while self.isFlying:
            time.sleep(self.loggingPeriod_in_ms / 2 / 1000)
        self.printDebug(f"</Condition> reached for [{self.current_state}]: is_Flying={self.isFlying}")
        return True

    def before_transition(self, event, state):
        self.current_transition = event
        return "before_transition_return"

    # This is for executing long-running UAV-actions
    def on_transition(self, event, state):
        self.printDebug(f"OT: On '{event}' at state '{state.id}' .")

        if event == "initialize":
            if self.uavOpStrategyImpl is None:
                raise Exception("No CF Operation Strategy selected!")
            if self.uavOpStrategyImpl.requires_scf() and not self.uavOpStrategyImpl.isSetSCF():
                raise Exception("SCF not initialized for CF operation strategy!")
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

        return "on_transition_return"

    def on_exit_state(self, event, state):
        self.printDebug(f"OnExState: Exiting '{state.id}' state from '{event}' event.")
        return "on_exit_state"

    def on_enter_state(self, event, state):
        self.printDebug(f"OnEnState: Entering '{state.id}' state from '{event}' event.")
        self.writeSMGraph()
        if state == self.hovering:
            self.printDebug("\tHovering now")
            # time.sleep(0.1)
        return "on_enter_state"

    def after_transition(self, event, state):
        self.printDebug(f"AT: After '{event}' at state '{state.id}'.")
        if state == self.flying and (event == "next_nav_goal" or event == "begin_nav_goal_sequence"):
            if len(self.targetPointsQueue) != 0:
                self.printDebug(f"\tNext Nav-Goal Count: {len(self.targetPointsQueue)}")
                self.next_nav_goal()
            else:
                self.printDebug("\tNav-Goals are empty.")
                self.keep_hovering()
        if state == self.landed:
            self.printDebug("\tState landed reached")
            self.landing_completed()
        self.writeSMGraph()
        return "after_transition"

    def writeSMGraph(self):
        asyncio.run(self.writePNG())

    # Generate the state diagram
    async def writePNG(self):
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


# This method implements a larger action sequence in the state machine.
# It  implements navigation for drones based on a set of coordinates.
# It wraps calls to the atomic action "navigate_to_simple" of a drone
def navigate_to_simple(drone: StateMachineDrone):
    if isinstance(drone.uavOpStrategyImpl.mc, PositionHlCommander):
        drone.printDebug(f"\tCurrent Position H1Commander: {drone.uavOpStrategyImpl.mc.get_position()}")
    if len(drone.targetPointsQueue) != 0:
        targetPoint = drone.targetPointsQueue.pop(0)

        # Possibility to modify coordinates here, using other estimates
        drone.uavOpStrategyImpl.navigate_to_simple(targetPoint=targetPoint)

        if isinstance(drone.uavOpStrategyImpl.mc, PositionHlCommander):
            drone.printDebug(f"\tCurrent Position H1Commander (LPS): {drone.uavOpStrategyImpl.mc.get_position()}")
        return
    drone.printDebug("\tAll Nav-Goals finished! keep hovering now.")
    drone.keep_hovering()
