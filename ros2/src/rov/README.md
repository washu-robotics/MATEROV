# Control ROV

## Installation

You may need to install `libserial-dev`:
```
sudo apt-get install libserial-dev
```

## Launch the ROV:

Launch joystick control for the ROV:
```
ros2 launch rov joystick.launch.py
```


This publishes `/joy/cmd_vel`. To subscribe to `/joy/cmd_vel` and control the ROV:


```
ros2 run motor_control thruster
```


(this should eventually be tied into `rov.launch.py`)

