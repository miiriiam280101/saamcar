#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation

scan_degrees = np.pi * 2


fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x,y)
plt.xlim(0,10)
plt.ylim(0,10)    

x, y = [], []


def callback(data):
    global x, y
    scan = list(data.ranges)

    step_size = scan_degrees / len(scan)
    print(len(scan))

    x1, y1 = [], []
    for i in range(len(scan)):
        alpha = data.angle_increment * i

        x1.append (np.cos(alpha) * scan[i])
        y1.append (np.sin(alpha) * scan[i])
    
    x = x1
    y = y1


    

def animate(i):
    sc.set_offsets(np.c_[x,y])
    
def listener():
    rospy.init_node('listener', anonymous=True)

    ani = matplotlib.animation.FuncAnimation(fig, animate, 
            frames=10, interval=100, repeat=True) 



    rospy.Subscriber("/scan", LaserScan, callback)

    plt.show()

    rospy.spin()

if __name__ == '__main__':
    listener()

