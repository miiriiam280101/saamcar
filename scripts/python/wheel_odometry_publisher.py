#!/usr/bin/env python
import numpy as np

# ROS includes
import rospy
import tf
from saamcar.msg import Encoder
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64

# globals
initialize = True
last_wheel_data = [0, 0] # [left, right]
wheel_velocity = [0, 0]
wheel_step_distance = 0.1 * np.pi / 60
wheel_distance = 0.23
position = [0, 0, 0] # x, y, theta

last_linear_speed = 0
linear_speed = 0
turn_speed = 0

current_time = None
last_time = None

odom_publisher = None
odom_broadcaster = None
vehicle_speed_publisher = None

def callback(data):
    global initialize, wheel_velocity, last_wheel_data, last_time, linear_speed, turn_speed, odom_publisher, odom_broadcaster, last_linear_speed
    if odom_broadcaster is None or odom_broadcaster is None:
        return

    current_time = rospy.get_time() # TODO: Check for Simulation Time (zero division)
    dt = current_time - last_time # calculate delta time

    if dt == 0:
        return

    if initialize:
        last_wheel_data[0] = data.left_wheel_data
        last_wheel_data[1] = data.right_wheel_data
        initialize = False
        rospy.loginfo("Initialized wheel_odometry")
        return

    # calculate linear wheel speed
    wheel_velocity[0] = (data.left_wheel_data - last_wheel_data[0])  * wheel_step_distance / dt
    wheel_velocity[1] = (data.right_wheel_data - last_wheel_data[1]) * wheel_step_distance / dt

    # include direction
    if data.left_wheel_dir == True: wheel_velocity[0] *= -1 
    if data.right_wheel_dir == True: wheel_velocity[1] *= -1 

    # calculate turn speed and linear vehicle speed
    linear_speed = np.mean(wheel_velocity)
    turn_speed = (wheel_velocity[1] - wheel_velocity[0]) / wheel_distance

    # calculate speed vector in global map
    theta = position[2]
    vx = (linear_speed * np.cos(theta))
    vy = (linear_speed * np.sin(theta))

    # add to position
    position[0] += vx * dt
    position[1] += vy * dt
    position[2] += turn_speed * dt

    ## set the position for easier use
    x = position[0] + 0.445 * np.sin(theta)
    y = position[1] + 0.445 * np.cos(theta)
    th = position[2]

    # publish odometry to ros #
    ros_current_time = rospy.Time.now()

    linear_speed_message = Float64()
    linear_speed_message.data = (linear_speed + last_linear_speed) / 2
    last_linear_speed = linear_speed

    odom = Odometry()
    odom.header.stamp = ros_current_time
    odom.header.frame_id = "odom"

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th) # works with python2
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    ## publish transform over tf
    # odom_broadcaster.sendTransform(
    # (x, y, 0.),
    # odom_quat,
    # ros_current_time,
    # "base_link",
    # "odom"
    # )

    ## set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, turn_speed))

    ## publish the message
    odom_publisher.publish(odom)
    vehicle_speed_publisher.publish(linear_speed_message)


    # set past timestep variables
    last_wheel_data[0] = data.left_wheel_data
    last_wheel_data[1] = data.right_wheel_data
    last_time = current_time
    
def listener():
    global last_time, current_time, odom_publisher, odom_broadcaster, vehicle_speed_publisher
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('base_odometry', anonymous=True)

    current_time = rospy.get_time()
    last_time = current_time
    
    odom_publisher = rospy.Publisher("/wheel_odometry/odom", Odometry, queue_size=100)
    vehicle_speed_publisher = rospy.Publisher("/wheel_odometry/vehicle_speed", Float64, queue_size=100)

    odom_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/base_sensor/wheel_data", Encoder, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
