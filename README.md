# Velocity & Steering Control for a Self Driving Car

## Overview
Design longitudinal (velocity) and lateral (steering) controller for a car to operate autonomously. Simulation done in Carla Simulator.  Reference velocity and waypoints are given for a race track.

<p align="center">
<img src="https://github.com/varunasthana92/Velocity_Steering_Control_Self_Drive_Car/blob/main/images/self_drive.gif" width = 400>
</p>

## Dependencies
* Python 3.6
* Carla Simulator
* matplotlib 2.2

## Implementation
* Implemented PID controller for the velocity control. Simulation was done in Carla Simulator.

<p align="center">
<img src="https://github.com/varunasthana92/Velocity_Steering_Control_Self_Drive_Car/blob/main/images/PID.png" width = 400>
</p>

* Implemented Stanley Controller for the steering control. Stanley Controller is based on Bicycle Model.
<p align="center">
<img src="https://github.com/varunasthana92/Velocity_Steering_Control_Self_Drive_Car/blob/main/images/stanley_controller.png" width = 450>
</p>

## Output
* Trajectory Control as compared to ground truth.
<p align="center">
<img src="https://github.com/varunasthana92/Velocity_Steering_Control_Self_Drive_Car/blob/main/images/result.png" width = 500>
</p>

* Speed as compared to ground truth
<p align="center">
<img src="https://github.com/varunasthana92/Velocity_Steering_Control_Self_Drive_Car/blob/main/images/speed.png" width = 500>
</p>

## Run Instructions
Clone the reppsotory inside the directory 
```
<CarlaSimulator/PythonClient/>
```
In a terminal
```
$ ./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30
```
In another terminal

```
$ python3.6 main.py
```

Result Verification

```
python3.6 verify.py racetrack_waypoints.txt /controller_output/trajectory.txt
```