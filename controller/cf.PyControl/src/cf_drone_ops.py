import time
from abc import ABC, abstractmethod

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from flask import Flask, jsonify, request
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine
from werkzeug.routing import BaseConverter, ValidationError

from cf_positioning import Point3D, PositionEstimationStrategy

try:
    import rclpy
    from rclpy.node import Node
    from crazyflie_interfaces.msg import GenericLogData
except Exception:
    # rclpy = None
    # Node = None
    raise RuntimeError(
        "rclpy Node error import"
    )

try:
    from crazyflie_interfaces.msg import Takeoff, Land, GoTo
except ImportError as e:
    #     Takeoff = Land = GoTo = None
    raise RuntimeError(
        "crazyflie_interfaces not found.\n"
        "Did you run:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  colcon build\n"
        "  source install/setup.bash"
    ) from e

DEFAULT_HEIGHT = 0.5
DEFAULT_VELOCITY = 0.3
BOX_LIMIT = 0.4


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


class CFOperationStrategy(ABC):
    _IS_DEBUG = False

    def __init__(self, scf: SyncCrazyflie | None = None, debug: bool = False):
        # super().__init__()
        self._IS_DEBUG = debug
        self.mc = None
        self._scf = scf

    def require_scf(self):
        if self._scf is None:
            raise RuntimeError(
                f"{self.__class__.__name__} requires a SyncCrazyflie instance"
            )

    def requires_scf(self) -> bool:
        return False

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

    def requires_scf(self) -> bool:
        return True

    def activate_idle_simple(self):
        # x=0.0, y=0.0, z=0.0 # initial position
        # controller=CONTROLLER_MELLINGER #=2
        # activate_mellinger_controller(cf=self._scf.cf)
        self.require_scf()
        self.printDebug("\tactivate_idle_simple")
        self.mc = PositionHlCommander(
            self._scf,
            default_height=DEFAULT_HEIGHT,
            default_velocity=DEFAULT_VELOCITY
        )
        self.printDebug("\tI am idling now!")

    def shutdown(self):
        self.require_scf()
        self.printDebug("shutdown")
        self.mc.stop()

    def take_off_simple(self):
        self.require_scf()
        self.printDebug(f"\ttake off simple")
        self.mc.take_off(velocity=DEFAULT_VELOCITY + 0.15)
        self.printDebug("Take off finished : I am hovering now")

    def landing_simple(self):
        self.require_scf()
        self.printDebug("I am landing now!")
        self.mc.land()
        self.printDebug("Technically, I am on the ground!")

    def navigate_to_simple(self, targetPoint: Point3D):
        self.require_scf()
        self.printDebug(f"\tNavigate to: {targetPoint.x}, {targetPoint.y}, {targetPoint.z}")
        self.mc.go_to(targetPoint.x, targetPoint.y, targetPoint.z, velocity=0.3)
        self.printDebug("\tNavigation finished")


class DebugLoggingCFOperationImpl(CFOperationStrategy):
    timeSleep = 0.1

    def activate_idle_simple(self):
        self.printDebug("\tactivate_idle_simple")
        # x=0.0, y=0.0, z=0.0 # initial position
        # controller=CONTROLLER_MELLINGER #=2
        self.mc = PositionHlCommander(self._scf, default_height=DEFAULT_HEIGHT, default_velocity=DEFAULT_VELOCITY)
        # activate_mellinger_controller(cf=self._scf.cf)
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


class RosTopicCFOperationImpl(CFOperationStrategy, Node):

    def __init__(self, cf_prefix='/cf0', debug=False, dronePosEst=PositionEstimationStrategy):
        if rclpy is None or Node is None:
            raise RuntimeError("ROS 2 not available")

        self.cf_prefix = cf_prefix.rstrip('/')  # normalize: '/cf0/' -> '/cf0'

        CFOperationStrategy.__init__(self, scf=None, debug=debug)
        self.cf_id = int(self.cf_prefix.replace('/cf', ''))
        Node.__init__(self, node_name=f'ds_cf_ops_{self.cf_id}')

        self.takeoff_pub = self.create_publisher(Takeoff, f'{self.cf_prefix}/takeoff', 10)
        self.land_pub = self.create_publisher(Land, f'{self.cf_prefix}/land', 10)
        self.goto_pub = self.create_publisher(GoTo, f'{self.cf_prefix}/go_to', 10)

        self.positionEstimator = dronePosEst

        self.is_flying = False
        self.state = {}
        self.subscription = self.create_subscription(
            GenericLogData,
            f'{self.cf_prefix}/state',
            self.cb,
            10
        )

    def cb(self, msg: GenericLogData):
        self.pm_vbat = msg.values[0]
        self.charge_current = msg.values[1]
        self.pm_state = int(msg.values[2])
        self.can_fly = bool(msg.values[3])
        self.is_flying = bool(msg.values[4])
        self.is_tumbled = bool(msg.values[5])

    def activate_idle_simple(self):
        self.printDebug("ROS idle (no-op)")

    def shutdown(self):
        self.printDebug("ROS shutdown (no-op)")

    def take_off_simple(self):
        msg = Takeoff()
        msg.group_mask = 0
        msg.height = DEFAULT_HEIGHT
        msg.use_current_yaw = True
        msg.duration.sec = 2
        self.takeoff_pub.publish(msg)
        self.wait_takeoff_landing_complete(target_z=DEFAULT_HEIGHT, isTakeOff=True)

    def landing_simple(self):
        msg = Land()
        msg.group_mask = 0
        msg.height = 0.0
        msg.duration.sec = 2
        self.land_pub.publish(msg)
        self.wait_takeoff_landing_complete(target_z=0.0, isTakeOff=False)

    def navigate_to_simple(self, targetPoint: Point3D):
        msg = GoTo()
        msg.group_mask = 0
        msg.relative = False
        msg.linear = False
        msg.goal.x = targetPoint.x
        msg.goal.y = targetPoint.y
        msg.goal.z = targetPoint.z
        msg.duration.sec = 2
        self.goto_pub.publish(msg)
        self.wait_navigation_complete(targetPoint)

    def wait_takeoff_landing_complete(
            self,
            target_z: float,
            timeout: float = 5.0,
            isTakeOff: bool = True,
            z_tol: float = 0.05
    ):
        start = time.time()
        while time.time() - start < timeout:
            z = self.positionEstimator.position_estimate[2]
            if isTakeOff:
                if self.is_flying and abs(z - target_z) < z_tol:
                    return True
            else:
                if not self.is_flying and abs(z - target_z) < z_tol:
                    return True

            time.sleep(0.05)

        return False

    def wait_navigation_complete(
            self,
            targetPoint: Point3D,
            timeout: float = 5.0,
            pos_tol: float = 0.05
    ):
        start = time.time()

        while time.time() - start < timeout:
            x, y, z = self.positionEstimator.position_estimate

            if (
                    abs(x - targetPoint.x) < pos_tol and
                    abs(y - targetPoint.y) < pos_tol and
                    abs(z - targetPoint.z) < pos_tol
            ):
                return True

            time.sleep(0.05)

        return False
