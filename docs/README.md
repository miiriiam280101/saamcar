## Car Architecture
The ADTF drivers of the car include the low-level arduino communication. There are 4 Arduinos on the car that control and sense. The `arduino_protocol.h` header file provided by their implementation, documents the interaction between the Arduinos an the computer. Each Arduino has a specific function (steer / motor actuation or reading IMU / Encoder data). Also each Arduino has an ID associated with it, defined in the `arduino_protocol.h`. 

The `base_actuator_node.cpp` publishes commands (steering & motor power) to the arduino.

The `base_sensor_node.cpp` node reads sensor packets, received from the Arduino responsible for IMU and Encoder data and publishes these in the ROS message format.

## SAAMcar Navigation
`roslaunch saamcar saamcar_navigation`

```
   SAAMcar navigation bootup sequence:
   
   1  launch bringup_saamcar_core.launch
      1.1-> launches SENSORS
      1.2-> launches STATIC TRANSFORMS
   2  launch bringup_cartographer 
      publishes map for the nav stack
```

## Remote Control
Due to issues with graphics and battery power, vnc is not a feasible option for remote control. The best way to control the car from an remote machine is through native ROS communication. 
Have a remote machine setup with ROS and the saamcar package. When having multible machines communicating in ROS, environment variables `ROS_IP` and `ROS_REMOTE_URI` have to be configured.

Using the car's WIFI as a hotspot following config could be used:

### CAR
```export ROS_REMOTE_URI=http://10.42.0.1:11311``` The ip address of the car (where rosmaster lives)

```export ROS_IP=10.42.0.1``` The ip address of the car

### REMOTE
```export ROS_REMOTE_URI=http://10.42.0.1:11311``` The ip address of the car (where rosmaster lives)

```export ROS_IP=10.42.0.2``` The ip address of the remote computer