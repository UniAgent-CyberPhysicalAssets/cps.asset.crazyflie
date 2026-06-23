#!/usr/bin/env python3
"""
Keyboard controller for cf.PyCtrl REST client.

Controls one Crazyflie through the AutoStateMachine client.

Mapping:
    W       -> move forward  (+x)
    S       -> move backward (-x)
    A       -> move left     (+y)
    D       -> move right    (-y)
    UP      -> move up       (+z)
    DOWN    -> move down     (-z)
    Q / ESC -> quit
    ENTER   -> activate idle
    SPACE   -> takeoff / landing

The controller reads the latest Crazyflie position from the cf.PyCtrl WebSocket
and sends a new absolute navigation target via the REST API.

Author: Dominik Grzelak
"""
import threading
from typing import Callable
import logging
from dataclasses import dataclass
from typing import Optional, Tuple, Callable

from pynput import keyboard

from auto_state_machine import AutoStateMachine


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)


@dataclass
class KeyboardControlConfig:
    step_xy: float = 0.25
    step_z: float = 0.15

    min_z: float = 0.25
    max_z: float = 1.0

    base_url: str = "http://127.0.0.1:5000"
    websocket_host: str = "127.0.0.1"
    websocket_port: int = 8765

    websocket_timeout: float = 2.0


class KeyboardDroneController:
    def __init__(self, config: KeyboardControlConfig):
        self.config = config
        self.running = True

        self.command_lock = threading.Lock()
        self.command_active = False
        self.command_thread = None

        self.client = AutoStateMachine(
            base_url=config.base_url,
            wshost=config.websocket_host,
            wsport=config.websocket_port
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.5

    def read_position(self) -> Optional[Tuple[float, float, float]]:
        position = self.client.read_position_websocket(
            timeout=self.config.websocket_timeout
        )

        if position is None:
            logger.warning("No WebSocket position received; using last local target")
            return None

        self.current_x, self.current_y, self.current_z = position
        return position

    def print_status(self):
        logger.info(
            f"Current target: "
            f"x={self.current_x:.3f}, "
            f"y={self.current_y:.3f}, "
            f"z={self.current_z:.3f}"
        )

    def activate_idle_command(self):
        logger.info("[ENTER] activate idle")
        success = self.client.activate_idle()

        if success:
            self.client.wait_for_state("idle", timeout=5.0)
            self.read_position()
            self.print_status()

    def send_target(self, x: float, y: float, z: float) -> bool:
        z = max(self.config.min_z, min(self.config.max_z, z))

        logger.info(
            f"Sending target: "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        success = self.client.navigate_to(x, y, z)

        if success:
            self.current_x = x
            self.current_y = y
            self.current_z = z
            self.print_status()
        else:
            logger.error("Failed to send keyboard navigation target")

        return success

    def toggle_takeoff_landing(self):
        current_state = self.client.get_current_state()
        logger.info(f"[SPACE] current state: {current_state}")

        if current_state == "idle":
            logger.info("[SPACE] begin takeoff")
            success = self.client.begin_takeoff()

            if success:
                self.client.wait_for_state("hovering", timeout=10.0)
                self.read_position()
                self.print_status()

            return

        if current_state == "hovering":
            logger.info("[SPACE] begin landing")
            success = self.client.begin_landing()

            if success:
                self.client.wait_for_state("idle", timeout=10.0)
                self.read_position()
                self.print_status()

            return

        logger.warning(
            f"[SPACE] ignored: takeoff/landing not allowed from state '{current_state}'"
        )

    def move_relative(self, dx: float, dy: float, dz: float):
        self.read_position()

        target_x = self.current_x + dx
        target_y = self.current_y + dy
        target_z = self.current_z + dz

        self.send_target(target_x, target_y, target_z)

    def move_forward(self):
        logger.info(f"[W] move forward +x by {self.config.step_xy} m")
        self.move_relative(dx=self.config.step_xy, dy=0.0, dz=0.0)

    def move_backward(self):
        logger.info(f"[S] move backward -x by {self.config.step_xy} m")
        self.move_relative(dx=-self.config.step_xy, dy=0.0, dz=0.0)

    def move_left(self):
        logger.info(f"[A] move left +y by {self.config.step_xy} m")
        self.move_relative(dx=0.0, dy=self.config.step_xy, dz=0.0)

    def move_right(self):
        logger.info(f"[D] move right -y by {self.config.step_xy} m")
        self.move_relative(dx=0.0, dy=-self.config.step_xy, dz=0.0)

    def move_up(self):
        logger.info(f"[UP] move up +z by {self.config.step_z} m")
        self.move_relative(dx=0.0, dy=0.0, dz=self.config.step_z)

    def move_down(self):
        logger.info(f"[DOWN] move down -z by {self.config.step_z} m")
        self.move_relative(dx=0.0, dy=0.0, dz=-self.config.step_z)

    def quit(self):
        logger.info("Quit keyboard controller")
        self.running = False
        return False

    def on_press(self, key):
        try:
            char = key.char

            if char is None:
                return None

            char = char.lower()

            if char == "w":
                return self.run_guarded_command("W", self.move_forward)
            elif char == "s":
                return self.run_guarded_command("S", self.move_backward)
            elif char == "a":
                return self.run_guarded_command("A", self.move_left)
            elif char == "d":
                return self.run_guarded_command("D", self.move_right)
            elif char == " ":
                return self.run_guarded_command("SPACE", self.toggle_takeoff_landing)
            elif char == "q":
                return self.quit()

        except AttributeError:
            if key == keyboard.Key.up:
                return self.run_guarded_command("UP", self.move_up)
            elif key == keyboard.Key.down:
                return self.run_guarded_command("DOWN", self.move_down)
            elif key == keyboard.Key.space:
                return self.run_guarded_command("SPACE", self.toggle_takeoff_landing)
            elif key == keyboard.Key.enter:
                return self.run_guarded_command("ENTER", self.activate_idle_command)
            elif key == keyboard.Key.esc:
                return self.quit()

        return None

    def run_guarded_command(self, command_name: str, command: Callable[[], object]):
        with self.command_lock:
            if self.command_thread is not None and self.command_thread.is_alive():
                logger.info(f"Ignored [{command_name}]: previous command still running")
                return None

            self.command_thread = threading.Thread(
                target=self._run_command_worker,
                args=(command_name, command),
                daemon=True
            )
            self.command_thread.start()

        return None

    def _run_command_worker(self, command_name: str, command: Callable[[], object]):
        try:
            logger.info(f"Started [{command_name}]")
            command()
        except Exception as e:
            logger.error(f"Command [{command_name}] failed: {e}")
        finally:
            logger.info(f"Finished [{command_name}]")

    def run(self):
        logger.info("Keyboard controller started")
        logger.info(
            "Use W/S/A/D + UP/DOWN. ENTER activates idle. "
            "SPACE toggles takeoff/landing. Press Q or ESC to quit."
        )
        logger.info(f"REST API: ws={self.config.base_url}")
        logger.info(
            f"WebSocket: ws://{self.config.websocket_host}:"
            f"{self.config.websocket_port}"
        )

        self.read_position()
        self.print_status()

        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

def main():
    config = KeyboardControlConfig(
        step_xy=0.25,
        step_z=0.15,
        min_z=0.15,
        max_z=2.00,
        base_url="http://127.0.0.1:5000",
        websocket_host="127.0.0.1",
        websocket_port=8765,
        websocket_timeout=2.0
    )

    controller = KeyboardDroneController(config)
    controller.run()


if __name__ == "__main__":
    main()