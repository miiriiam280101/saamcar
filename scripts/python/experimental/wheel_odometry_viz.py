#!/usr/bin/env python
import rospy
import numpy as np
from saamcar.msg import Encoder
import matplotlib.pyplot as plt
import matplotlib.animation
import numpy as np

fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x,y)
plt.xlim(0,10)
plt.ylim(0,10)    

initialize = True
last_wheel_data = [0, 0] # [left, right]
wheel_velocity = [0, 0]
wheel_step_distance = 0.1 * np.pi / 60
wheel_distance = 0.23
position = [0, 0, 0] # x, y, theta

linear_speed = 0
turn_speed = 0

current_time = None
last_time = None

def callback(data):
    global initialize, wheel_velocity, last_wheel_data, last_time, linear_speed, turn_speed, x, y
    current_time = rospy.get_time()
    dt = current_time - last_time # calculate delta time

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

    # append position for visuals
    x.append(position[0])
    y.append(position[1])

    last_wheel_data[0] = data.left_wheel_data
    last_wheel_data[1] = data.right_wheel_data
    last_time = current_time
    rospy.loginfo("wheel velocities: {} \n linear_speed: {:06.2} turn_speed: {:06.2} ".format(wheel_velocity, linear_speed, turn_speed))


def animate(i):
    sc.set_offsets(np.c_[x,y])
    
    
def listener():
    global last_time, current_time
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('base_odometry', anonymous=True)

    current_time = rospy.get_time()
    last_time = current_time
    
    rospy.Subscriber("/base_sensor/wheel_data", Encoder, callback)

    ani = matplotlib.animation.FuncAnimation(fig, animate, 
                frames=2, interval=100, repeat=True) 
    plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
