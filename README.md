# Simulation Interfaces Examples

## Sample script

This repository contains the example script using standardized simulation interfaces powering different simulation backends. The script implements the following scenario:
1. Initialize simulation
    1.1. Load world (only if supported)
    1.2. Spawn and despawn an object
    1.3. Spawn two robots and multiple static objects
2. Start the simulation
    2.1. Play and pause simulation
    2.2. Reset simulation to the initial state; move some objects
    2.3. Step simulation
3. Loop the simulation
   3.1. Query the entity state of a robot
   3.2. Send target goals to a robot and move it around
   3.3. Move the robot once it had reached a certain pose
   3.4. Move the robotic arm to simulate pick and place
4. Terminate the simulation (unload the world if supported)

The goal of this script is to demonstrate how to use the standardized simulation interfaces to control different simulation backends. Multiple parameters, such as robot speed, target poses, etc. are hardcoded for simplicity.

## Running the script

Run the simulator (Gazebo, Isaac Sim, or O3DE) and make sure the simulation interfaces are properly installed and sourced. 
Next, run the `warehouse_simulation_script.py` script with the desired backend flag.

| Simulator | flag                     |
| --------- | ------------------------ |
| Gazebo    | `--sim-backend gazebo`   |
| Isaac Sim | `--sim-backend isaacsim` |
| O3DE      | `--sim-backend o3de`     |

E.g. for Isaac Sim run:
```shell
python3 warehouse_simulation_script.py --sim-backend isaacsim
```

## Resources for simulation interfaces standard
See [the standard](https://github.com/ros-simulation/simulation_interfaces).

### UI & RViz2 plugin 
This plugin will allow you to conveniently, manually test your interfaces & the simulation:
https://github.com/RobotecAI/q_simulation_interfaces

### Worlds

### Robots

## Other resources
ROSCon 2025 talk introducing the standard and resources
