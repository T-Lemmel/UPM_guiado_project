# UPM_guiado_project

# Marvin: Patrol Robot for Senior Care Facilities

## Overview

This repository contains the implementation of a patrol robot, Marvin, designed for elder care facilities. Marvin autonomously navigates an apartment to detect emergency situations and communicate with medical staff. The project uses the Apolo simulation platform and Python for simulation and control.

## How to Run
1. Clone this repository.
2. Open `map.xml` in Apolo to load the simulation environment.
3. Run `main.py` to start the robot simulation.

### During the Simulation:
- The patient's position (landmark 0) is randomized within accessible areas.
- Paths, covariances, and intermediate points appear dynamically for visualization.
- The A* path is plotted when the robot backtracks to its starting position.

Close the visualization windows to continue the simulation.

## Dependencies
- Python 3.x
- Apolo Simulator ([GitHub Repository](https://github.com/mhernando/apolo))


## Contributors
- **Tom Lemmel**
- **Edoardo Selvaggi**

---

For more information, refer to the detailed [Project Report](./Guiado_Y_Navegacion_report.pdf).
