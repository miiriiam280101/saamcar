#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from PID import PID
import numpy

## clamping function for floats
clamp = lambda min_value, max_value, val : max(min(val, max_value), min_value)

steer_angle_publisher = None
speed_publisher = None
pid_enable_publisher = None
control_effort_publisher = None

speed_PID = None

# steering parameters
# max steer angle [left, right]
max_angle = 0.6981 # 40 degrees
max_steer_output = 0.85    # percentage sent to arduino


def calc_steer_setpoint(angle):
    angle = numpy.interp(angle, [-max_angle,max_angle], [-max_steer_output, max_steer_output])
    
    return -angle

def speed_callback(msg):
    control_effort = speed_PID(msg.data)
    print("Controll effort: {}".format(control_effort))
    control_effort_publisher.publish(control_effort)

def cmd_callback(data):
    # speed_publisher.publish(data.linear.x)
    speed_PID.setpoint = data.linear.x

    # calculate steer control signal
    steer_setpoint = calc_steer_setpoint(data.angular.z)
    steer_angle_publisher.publish(steer_setpoint)

    speed_publisher.publish(data.linear.x) # speed publisher publishes only the setpoint (base_actuator doesnt recieve the messages)
    
def run():
    global steer_angle_publisher, speed_publisher, pid_enable_publisher, control_effort_publisher, speed_PID
    rospy.init_node('base_controller')

    speed_PID = PID(Kp=30.0, Ki=25., Kd=0.0,
                    setpoint=0,
                    sample_time=0.025,
                    output_limits=(-30, 40),
                    auto_mode=True,
                    proportional_on_measurement=False,
                    integral_dewind=True)

    steer_angle_publisher = rospy.Publisher("/base_actuator/angle_setpoint", Float64, queue_size=100)
    speed_publisher = rospy.Publisher("/base_controller/speed_setpoint", Float64, queue_size=100)
    pid_enable_publisher = rospy.Publisher("/base_controller/pid_enable", Bool, queue_size=100)
    control_effort_publisher = rospy.Publisher("/base_actuator/control_effort_speed", Float64, queue_size=100)

    rospy.Subscriber("/base_controller/cmd_vel", Twist, cmd_callback)
    rospy.Subscriber("/wheel_odometry/vehicle_speed", Float64, speed_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    run()
