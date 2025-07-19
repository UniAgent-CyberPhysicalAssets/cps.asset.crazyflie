# [CPA] Crazyflie 2.x

This repository provides a cyber-physical representation of the **Crazyflie 2.x drone**, positioning it as an asset within the **Industry 4.0 ecosystem**. 

The resources here enable integration, simulation, and control of Crazyflie 2.x in both physical and virtual environments.

## Features

The repository includes the following components:

1. **Standalone High-Level Controller**:
   - A RESTful HTTP API, written in Python, to interact with Crazyflie 2.x (both physical and simulated instances).
   - See [cf.PyControl](https://github.com/UniAgent-CyberPhysicalAssets/cps.asset.crazyflie/tree/main/controller/cf.PyControl): A Terminal-based RESTful Controller for the Crazyflie 2.X
2. **Simulation Environments**:
   - A Gazebo/ROS-based simulation environment to test and validate Crazyflie 2.x in controlled virtual scenarios.
   - Using crazyswarm2 and sim_cf2 for simulation.
   - See [Usage Guide for sim_cf2](https://github.com/UniAgent-CyberPhysicalAssets/cps.asset.crazyflie/tree/main/simulation/sim_cf2-ext).
3. **Virtual Machine**:
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

## Outlook 

### Digital Twin

Development of the drone's digital twin is ongoing and will evolve as the relevant standards mature.

- **Asset Administration Shell (AAS):**
    A representation of the Crazyflie 2.x using Eclipse BaSyx Industrie 4.0 middleware. Multiple submodels will describe drone properties such as available actions, payload capacity, and physical characteristics.

- **Behavioral Runtime Model:**
    An executable Bigraphical Reactive System specification will simulate runtime behavior within cyber-physical environments.

> [!NOTE] 
> 
> This feature is under development and subject to change based on the finalization of AAS submodel standards:
> - Submodel Template: Unmanned aerial vehicle (Drone)
> - IDTA Number: 02081
> 
> See [here](https://industrialdigitaltwin.org/content-hub/teilmodelle) for more details.


## License

See each component's license file for details.

---

Copyright © 2025 The UniAgent Developers and Contributors.