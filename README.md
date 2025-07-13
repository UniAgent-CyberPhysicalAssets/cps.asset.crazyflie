# [CPA] Crazyflie 2.x

This repository provides a cyber-physical representation of the **Crazyflie 2.x drone**, positioning it as an asset within the **Industry 4.0 ecosystem**. 

The resources here enable integration, simulation, and control of Crazyflie 2.x in both physical and virtual environments.

## Features

The repository includes the following components:

1. **Standalone High-Level Controller**:
   - A RESTful HTTP API, written in Python, to interact with Crazyflie 2.x (both physical and simulated instances).
   - See [cf.PyControl](https://github.com/UniAgent-CyberPhysicalAssets/cps.asset.crazyflie/tree/main/controller/cf.PyControl): A Terminal-based RESTful Controller for the Crazyflie 2.X
2. **Digital Twin**:
   - **Asset Administration Shell (AAS) Standard**: Represents the drone's digital twin using the Eclipse BaSyx Industrie 4.0 middleware. It includes multiple submodels detailing various aspects of the Crazyflie 2.x, e.g., its atomic actions such as takeoff, navigating, and landing.
   - **Behavioral Runtime Model**: Includes an executable **Bigraphical Reactive System (BRS)** specification to simulate the drone's runtime behavior within a cyber-physical space.
3. **Simulation Environments**:
   - A Gazebo/ROS-based simulation environment to test and validate Crazyflie 2.x in controlled virtual scenarios.
   - Using crazyswarm2 and sim_cf2 for simulation.
   - See [Usage Guide for sim_cf2](https://github.com/UniAgent-CyberPhysicalAssets/cps.asset.crazyflie/tree/main/simulation/simulation-sim_cf2).
4. **Virtual Machine**:
   - A pre-configured virtual machine to simplify setup the drone across different development environments.
   - This is supplied by the developers of Bitcraze AB.

## Software Stack

This CPS asset relies on the following software stack:

- Python 3.7+ for the standalone high-level controller
- ROS Noetic or later for simulation environments
- BaSyx Middleware for implementing the AAS
- A virtualization platform (e.g., VirtualBox) for the provided virtual machine

Most components are provided as Docker containers for a quick start.

## Tutorial

For detailed instructions on how to set up and use this CPS asset,
please refer to the [documentation](https://uniagent-cyberphysicalassets.github.io/cps.asset.crazyflie/).