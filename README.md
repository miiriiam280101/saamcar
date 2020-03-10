# Getting Started

## Directories
- `config`    configuration files for various nodes
- `docs`      documentation
- `include`   header files for saamcar nodes
- `launch`    launch files (orchestrates multible nodes)
- `maps`      saved maps and cartographer .pbstream files (trajectory & submaps)
- `msg`       message definitions
- `scripts`   python scripts & -nodes
- `src`       cpp files

## Important commands
- `source ~/catkin_ws/devel/setup.sh` sets environment variables of the catkin workspace. This can be added to `bashrc`, to happen automatically.

- `roslaunch [package] [launchfile]` launches launchfiles from a package
    - `roslaunch saamcar saamcar_navigation`    bootsup car in nav mode
    - `roslaunch saamcar viz_navigation`        launches car's visualisation 

## Dependencies
`pylon_camera` (front_camera): https://github.com/magazino/pylon_camera

`usb_cam` (rear_camera) : `sudo apt install ros-melodic-usb-cam`

# Typical Problems
## Access Permission
The user typically has no permissions for `/dev/ttyACM*` Solution: add the user to the `dialout` group. Otherwise the node can not communicate with the MCUs.

## Rear Camera doesnt detatch
After using the `rear_camera` with the `usb_cam` module it cant be accessed again. **Unplug and Replug** the camera.

## base_actuator cant control motors
The cars motor controller is controlled by an PWM signal. If for some reason this PWM signal gets interrupted (remote turned off, arduino watchdog timeout) the controller will stop and wait for reinitialisation. Due to the implementation of the ADAS Arduinos the only way of reinitialization is to:
- Set the switch to `manual` controls and have `base_actuator` node started in the meanwhile
- Then use the remotes throttle until the wheels spin. Now the controller is reinitialized.
- Set the switch back to `autonomous` mode. Now the `base_actuator` node should be able to controls the car

 