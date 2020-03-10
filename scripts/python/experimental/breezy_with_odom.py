#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from roboviz import MapVisualizer
from saamcar.msg import Encoder
import numpy as np

MAP_SIZE_PIXELS = 4000
MAP_SIZE_METERS = 75
LIDAR_ARRAY_SIZE = 360


# slam stuff
class RpLidar(Laser):
    '''
    A class for the SLAMTEC RPLidar A1
    '''
    def __init__(self, detectionMargin = 0, offsetMillimeters = 0):
        
        Laser.__init__(self, LIDAR_ARRAY_SIZE, 10, 360, 1200, detectionMargin, offsetMillimeters)


mapbytes = bytearray(MAP_SIZE_PIXELS ** 2)
slam = RMHC_SLAM(RpLidar(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, sigma_xy_mm=100, sigma_theta_degrees=10, max_search_iter=1000 ) 
viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
pos = [0, 0, 0]
odom = (0, 0, 0)
current_time_breezy = None
last_time_breezy = None
last_linear_vel = []
last_rot_vel = []


# odom stuff
initialize = True
last_wheel_data = [0, 0] # [left, right]
wheel_velocity = [0, 0]
wheel_step_distance = 0.1 * np.pi / 60
wheel_distance = 0.23
position = [0, 0, 0] # x, y, theta

linear_speed = 0
turn_speed = 0
dt = 0

current_time_odom = None
last_time_odom = None

# callbacks
def callback_breezy(data):
    # include global variables
    global slam, mapbytes, pos, last_time_breezy, current_time_breezy, last_linear_vel, last_rot_vel

    # calculate delta times
    current_time_breezy = rospy.get_time()
    dt = current_time_breezy - last_time_breezy

    # convert scan into right scale(mm) 
    scan = list(np.array(data.ranges) * 1000)

    # This will swap the laser measurement degrees to front. DUe to the nature of breezy slam regards handling
    # laser measurements
    scan_angles = [((data.angle_increment / np.pi * 180 * i) + 180) % 360 for i in range(len(scan))]
    #scan_angles = list(np.ones(len(scan)) * data.angle_increment / np.pi * 180)
    # print(scan_angles)

    # print("Scan length: {}, angle_incement: {}".format(len(scan), data.angle_increment))

    if len(last_linear_vel) > 0:
        odom = (np.mean(last_linear_vel) * 100, np.mean(last_rot_vel), dt)
        last_linear_vel = []
        last_rot_vel = []
    else:
       odom = (0, 0, 0)  
    print("odom: {}".format(odom))

    # if( abs(odom[1]) > 0.1):
    #     slam.sigma_xy_mm=50
    #     slam.sigma_theta_degrees=15
    #     slam.max_search_iter=5000
    # else:
    #     slam.sigma_xy_mm=10
    #     slam.sigma_theta_degrees=10
    #     slam.max_search_iter=1000 

    #slam.update(scan, scan_angles_degrees=scan_angles)
    slam.update(scan, pose_change=(odom), scan_angles_degrees=scan_angles)    
   
    #slam.update(scan, scan_angles_degrees=scan_angles, should_update_map=True)
    pos[0], pos[1], pos[2],  = slam.getpos()
    slam.getmap(mapbytes)

    
    last_time_breezy = current_time_breezy
    #print("Position X, Y, THETA: {}".format(pos))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s")

def callback_odom(data):
    global initialize, wheel_velocity, last_wheel_data, last_time_odom, linear_speed, turn_speed, dt, last_linear_vel, last_rot_vel
    current_time_odom = rospy.get_time()
    dt = current_time_odom - last_time_odom # calculate delta time

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
    if data.left_wheel_dir  == True: wheel_velocity[0] *= -1 
    if data.right_wheel_dir == True: wheel_velocity[1] *= -1 

    # calculate turn speed and linear vehicle speed
    linear_speed = np.mean(wheel_velocity)
    turn_speed = (wheel_velocity[1] - wheel_velocity[0]) / wheel_distance

    # add speeds to filter array
    last_linear_vel.append(linear_speed)
    last_rot_vel.append(turn_speed)

    # calculate speed vector in global map
    theta = position[2]
    vx = (linear_speed * np.cos(theta))
    vy = (linear_speed * np.sin(theta))

    # add to position
    position[0] += vx * dt
    position[1] += vy * dt
    position[2] += turn_speed * dt

    last_wheel_data[0] = data.left_wheel_data
    last_wheel_data[1] = data.right_wheel_data
    last_time_odom = current_time_odom
    #rospy.loginfo("wheel velocities: {} \n linear_speed: {:06.2} turn_speed: {:06.2} ".format(wheel_velocity, linear_speed, turn_speed))

    
def listener():
    global last_time_odom, current_time_odom, current_time_breezy, last_time_breezy
    rospy.init_node('listener', anonymous=True)

    current_time_odom = rospy.get_time()
    last_time_odom = current_time_odom
    current_time_breezy = rospy.get_time()
    last_time_breezy = current_time_breezy

    rospy.Subscriber("/scan", LaserScan, callback_breezy)
    rospy.Subscriber("/base_sensor/wheel_data", Encoder, callback_odom)
    
    while not rospy.is_shutdown():
        viz.display(pos[0]/1000., pos[1]/1000., pos[2], mapbytes)
        #rospy.spin()

if __name__ == '__main__':
    listener()

