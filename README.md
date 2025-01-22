# Cyber-physical Asset: Crazyflie 2.x 

This repository provides a cyber-physical representation of the **Crazyflie 2.x drone**, positioning it as an asset within the **Industry 4.0 ecosystem**. 

The resources here enable integration, simulation, and control of Crazyflie 2.x in both physical and virtual environments.

## Features

The repository includes the following components:

1. **Standalone High-Level Controller**:
   - A RESTful HTTP API, written in Python, to interact with Crazyflie 2.x (both physical and simulated instances). The digital twin makes use of this.
2. **Digital Twin**:
   - **Asset Administration Shell (AAS)**: Represents the drone's digital twin using the Eclipse BaSyx Industrie 4.0 middleware. It includes multiple submodels detailing various aspects of the Crazyflie 2.x, e.g., its atomic actions such as takeoff, navigating, and landing.
   - **Behavioral Runtime Model**: Implements a **Bigraphical Reactive System (BRS)** to simulate the drone's runtime behavior within a cyber-physical space.
3. **Simulation Environments**:
   - A ROS-based simulation framework to test and validate Crazyflie 2.x in controlled virtual scenarios.
4. **Virtual Machine**:
   - A pre-configured virtual machine to simplify setup the drone across different development environments.

## Prerequisites

Before using this CPS asset, ensure the following dependencies are installed:

- Python 3.7+ for the standalone high-level controller
- ROS Noetic or later for simulation environments
- BaSyx Middleware for implementing the AAS
- A virtualization platform (e.g., VirtualBox) for the provided virtual machine