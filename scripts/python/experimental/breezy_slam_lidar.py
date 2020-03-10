#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from roboviz import MapVisualizer
import numpy as np

MAP_SIZE_PIXELS = 800
MAP_SIZE_METERS = 100
LIDAR_ARRAY_SIZE = 300

class RpLidar(Laser):
    '''
    A class for the SLAMTEC RPLidar A1
    '''
    def __init__(self, detectionMargin = 0, offsetMillimeters = 0):
        
        Laser.__init__(self, LIDAR_ARRAY_SIZE, 10, 360, 12000, detectionMargin, offsetMillimeters)


mapbytes = bytearray(MAP_SIZE_PIXELS ** 2)
slam = RMHC_SLAM(RpLidar(), MAP_SIZE_PIXELS, MAP_SIZE_METERS) 
viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
pos = [0, 0, 0]

def callback(data):
    global slam, mapbytes, pos

    scan = list(np.array(data.ranges) * 1000)[0:300]
    print(len(data.ranges))
    #scan_angles = list(np.ones(len(scan)) * data.angle_increment / np.pi * 180)

    print("Scan length: {}, angle_incement: {}".format(len(scan), data.angle_increment))

    slam.update(scan)
    #slam.update(scan, scan_angles_degrees=scan_angles, should_update_map=True)
    pos[0], pos[1], pos[2],  = slam.getpos()
    slam.getmap(mapbytes)

    print("Position X, Y, THETA: {}".format(pos))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s")
    
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, callback)
    
    while not rospy.is_shutdown():
        viz.display(pos[0]/1000., pos[1]/1000., pos[2], mapbytes)
        #rospy.spin()

if __name__ == '__main__':
    listener()

