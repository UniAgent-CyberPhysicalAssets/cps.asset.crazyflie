import sys
import time
from abc import ABC, abstractmethod

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.mem import CompressedSegment
from cflib.crazyflie.mem import CompressedStart
from cflib.crazyflie.mem import MemoryElement
from cflib.utils import uri_helper
from flask import Flask, jsonify, request
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine
from werkzeug.routing import BaseConverter, ValidationError

from cf_ops_util import activate_mellinger_controller, upload_trajectory, rotate_beizer_node, create_circle_trajectory, \
    build_trajectory
from cf_positioning import Point3D, PositionEstimationStrategy

try:
    import rclpy
    from rclpy.node import Node
    from crazyflie_interfaces.msg import GenericLogData
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Point
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

DEFAULT_HEIGHT = 0.55
DEFAULT_VELOCITY = 0.3
BOX_LIMIT = 0.4


class CFOperationStrategy(ABC):
    _IS_DEBUG = False

    def __init__(self, scf: SyncCrazyflie | None = None, debug: bool = False, console=None):
        # super().__init__()
        self._scf = scf
        self.mc = None
        self._IS_DEBUG = debug
        self.console = console

    def require_scf(self):
        if self._scf is None:
            raise RuntimeError(
                f"{self.__class__.__name__} requires a SyncCrazyflie instance"
            )
        self._scf.cf.platform.send_arming_request(True)

    def requires_scf(self) -> bool:
        return False

    def isSetSCF(self):
        return self._scf is not None

    def getSCF(self):
        return self._scf

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
    def take_off_simple(self, height=DEFAULT_HEIGHT, velocity=DEFAULT_VELOCITY):
        pass

    @abstractmethod
    def landing_simple(self):
        pass

    @abstractmethod
    def navigate_to_simple(self, targetPoint: Point3D):
        pass

    @abstractmethod
    def run_sequence(self, sequence, origin_x, origin_y, origin_z, origin_yaw, loops: int = 1):
        pass


class HlCommanderCFOperationImpl(CFOperationStrategy):

    def requires_scf(self) -> bool:
        return True

    def activate_idle_simple(self):
        self.require_scf()
        self.printDebug("\tactivate_idle_simple()")
        # activate_mellinger_controller(cf=self._scf.cf)
        self.mc = PositionHlCommander(
            self._scf,
            default_height=DEFAULT_HEIGHT,
            default_velocity=DEFAULT_VELOCITY
        )
        self._scf.cf.platform.send_arming_request(True)
        self.printDebug("\tI am idling now!")

    def shutdown(self):
        self.require_scf()
        self.printDebug("Shutdown")
        self.mc.stop()

    def take_off_simple(self, height=None, velocity=None):
        self.require_scf()
        self.printDebug("\ttake_off_simple()")
        time.sleep(1)
        if height is None:
            height = DEFAULT_HEIGHT
        if velocity is None:
            velocity = DEFAULT_VELOCITY

        self.mc.take_off(height=height, velocity=velocity)
        self.printDebug("finished:take_off_simple()")


    def landing_simple(self):
        self.require_scf()
        self.printDebug("landing_simple()")
        self.mc.land()
        # self.getSCF().cf.high_level_commander.land(0.0, 2.0)
        self.printDebug("finished:landing_simple()")

    def navigate_to_simple(self, targetPoint: Point3D):
        self.require_scf()
        self.printDebug(f"\tnavigate_to_simple(): {targetPoint.x}, {targetPoint.y}, {targetPoint.z}")
        self.mc.go_to(targetPoint.x, targetPoint.y, targetPoint.z, velocity=DEFAULT_VELOCITY)
        self.printDebug("\tfinished:navigate_to_simple()")

    def run_sequence(self, sequence, origin_x, origin_y, origin_z, origin_yaw, loops: int = 1):
        self.require_scf()
        self.printDebug(f"\trun_sequence(): {sequence}")
        cf = self.getSCF().cf

        # Prepare trajectory and upload it to Crazyflie
        trajectory = build_trajectory(sequence)
        # trajectory = self.create_trajectory(0.0, r1)

        relative = True

        duration = upload_trajectory(cf, 1, trajectory)
        self.printDebug(f"run_sequence():duration: {duration}")

        # Arm the Crazyflie
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Start Trajectory
        commander = cf.high_level_commander
        for i in range(loops):
            commander.start_trajectory(trajectory_id=1, time_scale=1.0, relative_position=relative, relative_yaw=False)
            time.sleep(duration)
            time.sleep(0.1)

        # commander.stop()


class DebugLoggingCFOperationImpl(CFOperationStrategy):
    timeSleep = 0.1

    def activate_idle_simple(self):
        self.printDebug("\tactivate_idle_simple")
        self.mc = PositionHlCommander(self._scf, default_height=DEFAULT_HEIGHT, default_velocity=DEFAULT_VELOCITY)
        time.sleep(self.timeSleep)
        self.printDebug("\tI am idling now!!")

    def shutdown(self):
        self.printDebug("shutdown")
        self.mc.stop()

    def take_off_simple(self, height=DEFAULT_HEIGHT, velocity=DEFAULT_VELOCITY):
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

    def run_sequence(self, sequence, origin_x, origin_y, origin_z, origin_yaw, loops: int = 1):
        self.printDebug(f"\trun_sequence: {sequence}")
        time.sleep(self.timeSleep)
        self.printDebug("\trun_sequence")


# for dscf simulator
class RosTopicCFOperationImpl(CFOperationStrategy, Node):

    def __init__(self, cf_prefix='/cf0', debug=False, dronePosEst=PositionEstimationStrategy, console=None):
        if rclpy is None or Node is None:
            raise RuntimeError("ROS 2 not available")

        self.cf_prefix = cf_prefix.rstrip('/')  # normalize: '/cf0/' -> '/cf0'

        CFOperationStrategy.__init__(self, scf=None, debug=debug, console=console)
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

    def take_off_simple(self, height=None, velocity=None):
        h0 = height
        v0 = velocity
        if h0 is None:
            h0 = DEFAULT_HEIGHT
        if v0 is None:
            v0 = DEFAULT_VELOCITY

        msg = Takeoff()
        msg.group_mask = 0
        msg.height = h0
        msg.use_current_yaw = True
        msg.duration.sec = 2
        self.takeoff_pub.publish(msg)
        self.wait_takeoff_landing_complete(target_z=h0, isTakeOff=True)

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

    def run_sequence(self, sequence, origin_x, origin_y, origin_z, origin_yaw, loops: int = 1):
        """
        Execute a compressed trajectory using only GoTo messages.

        New supported input (array-length=1 only):
          - sequence: list of dict elements with:
              - {"type":"start","x":..,"y":..,"z":..,"yaw":..}
              - {"type":"segment","duration":..,"x":[dx], "y":[dy], "z":[dz], "yaw":[dyaw]}

        Mapping (because we only support array length 1):
          - Each segment becomes one GoTo.
          - Treat x/y/z as per-segment relative increments (dx, dy, dz) in meters.
          - Treat yaw as per-segment relative increment (dyaw) in degrees unless you decide otherwise.
          - Convert to absolute goal using origin_* + accumulated relative offsets.
        """

        self.printDebug("ROS run_sequence() started")
        self.printDebug(f"\tOrigin: x={origin_x}, y={origin_y}, z={origin_z}, yaw={origin_yaw}")
        self.printDebug(f"\tElements: {len(sequence) if sequence else 0}")

        if not sequence:
            raise ValueError("Empty sequence")

        # Yaw in degrees for GoTo
        yaw_deg = float(origin_yaw)

        # Optional start element
        idx = 0
        first = sequence[0]
        if isinstance(first, dict) and first.get("type") == "start":
            idx = 1

        # Each segment -> one GoTo
        for seg_i in range(idx, len(sequence)):
            seg = sequence[seg_i]
            if not isinstance(seg, dict) or seg.get("type") != "segment":
                raise ValueError(f"Invalid trajectory element at index {seg_i}: {seg}")

            duration_s = float(seg.get("duration", 0.0))
            if duration_s <= 0.0:
                raise ValueError(f"Invalid duration at index {seg_i}: {duration_s}")

            # New format: x/y/z/yaw each are arrays of length 0 or 1
            def _get_coeff(axis: str, default: float = 0.0) -> float:
                arr = seg.get(axis, None)
                if arr is None:
                    return default
                if not isinstance(arr, list):
                    raise ValueError(f"Invalid '{axis}' (expected list) at index {seg_i}: {arr}")
                if len(arr) == 0:
                    return default
                if len(arr) != 1:
                    raise ValueError(f"Only array length 0 or 1 supported for '{axis}' at index {seg_i}: {arr}")
                return float(arr[0])

            dx = _get_coeff("x", 0.0)
            dy = _get_coeff("y", 0.0)
            dz = _get_coeff("z", 0.0)
            dyaw = _get_coeff("yaw", 0.0)

            # Absolute target
            ax = float(origin_x) + dx
            ay = float(origin_y) + dy
            az = float(origin_z) + dz
            ayaw = float(yaw_deg) + dyaw

            self.printDebug(
                f"Segment {seg_i}: dt={duration_s}s "
                f"rel=({dx},{dy},{dz}),(yaw={dyaw}) -> abs=({ax},{ay},{az}),(yaw={ayaw}) "
            )

            msg = GoTo()
            msg.group_mask = 0
            msg.relative = False
            msg.linear = False

            msg.goal = Point(x=ax, y=ay, z=az)
            msg.yaw = float(ayaw)  # degrees

            dur = Duration()
            dur.sec = int(duration_s)
            dur.nanosec = int((duration_s - int(duration_s)) * 1e9)
            msg.duration = dur

            self.goto_pub.publish(msg)

            ok = self.wait_navigation_complete(Point3D(ax, ay, az))
            if not ok:
                self.printDebug(f"Segment {seg_i} target not reached in time")
                return False

            self.printDebug(f"Segment {seg_i} reached")

        self.printDebug("ROS run_sequence() completed successfully")
        return True

    # Helper
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

    # Helper
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
