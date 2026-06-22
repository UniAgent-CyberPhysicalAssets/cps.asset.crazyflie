#!/usr/bin/env python3
"""
Client orchestrator for multiple cf.PyCtrl REST APIs.

This class drives one or more Crazyflie controller instances through their REST endpoints and executes predefined state-machine sequences, such as:

- `activate_idle -> begin_takeoff -> navigate-> begin_landing`

Thus, this specific "sequence executor" (execute_sequence) assumes that the corresponding cf.PyCtrl servers are already running.


Author: Tianxiong Zhang

Maintainer: Dominik Grzelak
"""
import json
import logging
import threading
import time
from pathlib import Path
from typing import List, Dict, Any, Optional

import requests

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class AutoStateMachine:
    def __init__(self, base_url: str = "http://127.0.0.1:5000"):  #5001 5002 ...
        """Initialize the automatic state machine controller

        Args:
            base_url: Base URL of the API server
        """
        self.base_url = base_url
        self.current_state = None
        self.navigation_points = []

    def get_current_state(self) -> str:
        """Get current state"""
        try:
            response = requests.get(f"{self.base_url}/status")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                return self.current_state
            else:
                logger.error(f"Failed to get status: {response.status_code}")
                return None
        except Exception as e:
            logger.error(f"An error occurred while getting the status: {str(e)}")
            return None

    def activate_idle(self) -> bool:
        """Activate idle state"""
        try:
            response = requests.post(f"{self.base_url}/activate_idle")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Activated idle state, current state: {self.current_state}")
                return True

            response_text = response.text or ""

            if "activate_idle when in IDLE" in response_text or "activate_idle when in HOVERING" in response_text:
                self.current_state = "idle"
                logger.info("Drone is already in idle/hovering state; treating activate_idle as successful")
                return True

            logger.error(f"Failed to activate idle state: {response.status_code}")
            logger.error(response_text)
            return False
        except Exception as e:
            logger.error(f"An error occurred while activating the idle state: {str(e)}")
            return False

    def begin_takeoff(self) -> bool:
        """Start takeoff"""
        try:
            response = requests.post(f"{self.base_url}/begin_takeoff")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Takeoff started, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to start takeoff: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while starting takeoff: {str(e)}")
            return False

    def begin_landing(self) -> bool:
        """Start landing"""
        try:
            response = requests.post(f"{self.base_url}/begin_landing")
            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Landing started, current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to start landing: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while starting landing: {str(e)}")
            return False

    def fly_trajectory(self, trajectory_file: Optional[str] = None, payload: Optional[Dict[str, Any]] = None) -> bool:
        """Execute a trajectory via the /fly-trajectory endpoint.

        Args:
            trajectory_file: Optional path to a JSON trajectory file.
            payload: Optional trajectory payload dictionary. If both are provided, payload wins.

        Returns:
            bool: Whether the trajectory command was accepted successfully.
        """
        try:
            if payload is None:
                if trajectory_file is None:
                    raise ValueError("trajectory_file or payload must be provided")

                trajectory_path = Path(trajectory_file)
                with trajectory_path.open("r", encoding="utf-8") as f:
                    payload = json.load(f)

            response = requests.post(
                f"{self.base_url}/fly-trajectory",
                json=payload,
                headers={"Content-Type": "application/json"}
            )

            if response.status_code == 200:
                data = response.json()
                self.current_state = data.get("state")
                sequence_len = len(payload.get("sequence", []))
                logger.info(
                    f"Trajectory command accepted ({sequence_len} elements), "
                    f"current state: {self.current_state}"
                )
                return True

            logger.error(f"Failed to execute trajectory: {response.status_code}")
            logger.error(response.text)
            return False

        except Exception as e:
            logger.error(f"An error occurred while executing trajectory: {str(e)}")
            return False

    def navigate_to(self, x: float, y: float, z: float) -> bool:
        """Navigate to specified position

        Args:
            x: x coordinate
            y: y coordinate
            z: z coordinate

        Returns:
            bool: Whether the navigation command was sent successfully
        """
        try:
            response = requests.post(f"{self.base_url}/navigate/{x}/{y}/{z}")

            if response.status_code == 200:
                self.current_state = response.json()["state"]
                logger.info(f"Navigation command sent to position ({x}, {y}, {z}), current state: {self.current_state}")
                return True
            else:
                logger.error(f"Failed to send navigation command: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"An error occurred while sending navigation command: {str(e)}")
            return False

    def add_navigation_point(self, x: float, y: float, z: float) -> bool:
        """Add a single navigation point to the queue

        Args:
            x: x coordinate
            y: y coordinate
            z: z coordinate

        Returns:
            bool: Whether the point was added successfully
        """
        try:
            self.navigation_points.append({"x": x, "y": y, "z": z})
            return True
        except Exception as e:
            logger.error(f"Failed to add navigation point: {str(e)}")
            return False

    def add_navigation_points(self, points: List[Dict[str, float]]) -> bool:
        """Add multiple navigation points at once

        Args:
            points: List of dictionaries containing x, y, z coordinates

        Returns:
            bool: Whether all points were added successfully
        """
        try:
            time.sleep(0.5)  # 1 -> 0.5

            for point in points:
                x, y, z = point['x'], point['y'], point['z']

                if x < 0 or y < 0 or z < 0:
                    payload = {"x": x, "y": y, "z": z}
                    response = requests.post(f"{self.base_url}/navigate/append", json=payload)
                else:
                    response = requests.post(
                        f"{self.base_url}/navigate/append/{x}/{y}/{z}"
                    )

                if response.status_code != 200:
                    logger.error(f"Failed to add navigation point: {point}")
                    return False
                logger.info(f"Added navigation point: ({x}, {y}, {z})")
                time.sleep(0.2)
            return True
        except Exception as e:
            logger.error(f"An error occurred while adding navigation points: {str(e)}")
            return False

    def wait_for_state(self, target_state: str, timeout: float = 10.0) -> bool:
        """Wait for state machine to transition to specified state

        Args:
            target_state: Target state
            timeout: Timeout in seconds

        Returns:
            bool: Whether the target state was reached
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_state = self.get_current_state()
            if current_state == target_state:
                logger.info(f"Target state reached: {target_state}")
                return True
            time.sleep(0.1)
        logger.error(f"Waiting for state {target_state} timed out (current_state={self.get_current_state()}")
        return False

    def execute_sequence(self, sequence: List[Dict[str, Any]], batch_navigation: bool = False) -> bool:
        """Execute state machine transition sequence

        Args:
            sequence: Transition sequence, each element is a dictionary containing:
                - action: Action to execute (activate_idle/begin_takeoff/begin_landing/navigate)
                - wait_state: State to wait for
                - timeout: Wait timeout (optional)
                - wait_time: Optional additional wait time in this state
                - x/y/z: Navigation target coordinates (only for navigate action)
                - trajectory_file or payload: Trajectory input (only for fly_trajectory action)
            batch_navigation: If True, send all navigation points at once. If False, send one point at a time.

        Returns:
            bool: Whether the sequence executed successfully
        """
        self.navigation_points = []
        for step in sequence:
            if step["action"] == "navigate":
                self.add_navigation_point(step["x"], step["y"], step["z"])

        nav_index = 0
        i = 0
        while i < len(sequence):
            step = sequence[i]
            action = step["action"]
            wait_state = step["wait_state"]
            timeout = step.get("timeout", 10.0)

            if action == "activate_idle":
                success = self.activate_idle()
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "begin_takeoff":
                success = self.begin_takeoff()
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "begin_landing":
                success = self.begin_landing()
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "fly_trajectory":
                success = self.fly_trajectory(
                    trajectory_file=step.get("trajectory_file"),
                    payload=step.get("payload")
                )
                if not success or not self.wait_for_state(wait_state, timeout):
                    return False
            elif action == "navigate":
                if batch_navigation:
                    if nav_index == 0:
                        if len(self.navigation_points) > 1:
                            points_to_add = self.navigation_points[:-1]
                            if not self.add_navigation_points(points_to_add):
                                logger.error("Failed to add navigation points")
                                return False
                        if len(self.navigation_points) > 0:
                            last_point = self.navigation_points[-1]
                            success = self.navigate_to(last_point["x"], last_point["y"], last_point["z"])
                            if not success:
                                return False
                        if not self.wait_for_state("hovering", timeout):
                            logger.error("Failed to reach hovering state after batch navigation")
                            return False
                        last_navigate_index = max(idx for idx, s in enumerate(sequence) if s["action"] == "navigate")
                        i = last_navigate_index + 1
                        nav_index += 1
                        continue
                    else:
                        i += 1
                        nav_index += 1
                        continue
                else:
                    if nav_index < len(self.navigation_points):
                        point = self.navigation_points[nav_index]
                        logger.info(f"Sending navigation point {nav_index+1}/{len(self.navigation_points)}: ({point['x']}, {point['y']}, {point['z']})")
                        current_state = self.get_current_state()
                        logger.info(f"Current state before navigation: {current_state}")
                        success = self.navigate_to(point["x"], point["y"], point["z"])
                        if not success:
                            return False
                        if not self.wait_for_state("hovering", timeout):
                            logger.error("Failed to reach hovering state after navigation")
                            return False
                        if "wait_time" in step:
                            logger.info(f"Waiting for additional {step['wait_time']} seconds")
                            time.sleep(step["wait_time"])
                            logger.info(f"Additional wait time completed")
                        nav_index += 1
                    else:
                        logger.error(f"Invalid navigation index: {nav_index}")
                        return False
            else:
                logger.error(f"Unknown action: {action}")
                return False
            if "wait_time" in step and action != "navigate":
                logger.info(f"Waiting for additional {step['wait_time']} seconds")
                time.sleep(step["wait_time"])
                logger.info(f"Additional wait time completed")
            i += 1
        return True
