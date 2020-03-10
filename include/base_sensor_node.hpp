#pragma once

#include <math.h>

#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

/* Messages */
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include <saamcar/Encoder.h>

#include "arduino_com_client.h"

#include <boost/geometry/geometries/adapted/boost_array.hpp> 

class BaseSensorNode
{
    private:    
        arduino_com_client rear_imu_wheel_device;
        //arduino_com_client rear_us_device;

        /* ROS Publisher */
        ros::Publisher imu_publisher;

        ros::Publisher wheel_data_right_publisher;

        ros::Publisher wheel_data_publisher;

        /* ROS Rate */
        //ros::Rate rate(40);


        /* do Serial comm */
        void poll();

        /* processing functions of sensor packets */
        void processIMU(uint32_t timestamp, tDataUnion data);
        void processENC(uint32_t timestamp, tDataUnion data, SENSOR_ID id);

        /* processing function of Odometry data */
        void processOdom();


        /* wheelencoder last recieved packet timestamp */
        int left_wheel_timestamp = 0;
        int right_wheel_timestamp = 0;
        uint32_t left_wheel_data, right_wheel_data, last_left_wheel_data, last_right_wheel_data;
        bool     init_wheel_data = true;
        bool     valid_wheel_packet, left_wheel_dir, right_wheel_dir;
        uint8_t  init_garbage_counter = 0;

        /* left and right Frames arrive typically within 200 us. 
           The next encoder Packet arround 25 ms. This threshold
           masks the data */
        uint32_t wheel_packet_timeout = 400; 

        ros::Time last_time, current_time;


        // double x = 0.0;
        // double y = 0.0;
        // double th = 0.0;
        // double vr, vl, vth;
    /*  sensor_msgs::JointState joint_states;
        geometry_msgs::TransformStamped robot_tf;
        ros::Publisher *joint_state_pub;
        char *name[2] = {"rear_left_wheel_hinge", "rear_right_wheel_hinge"};
        float pos[2] = {0, 0};
        float vel[2] = {0, 0};
        float eff[2] = {0, 0}; */


        boost::array<double, 9UL> undefined_array = {-1., 0., 0., 
                                                      0., 0., 0., 
                                                      0., 0., 0.,};


        boost::array<double, 9UL> none_array = {0., 0., 0., 
                                                0., 0., 0., 
                                                0., 0., 0.,};

        boost::array<double, 9UL> imu_angular_covariance = {0.4, 0., 0., 
                                                            0., 0.4, 0., 
                                                            0., 0., 0.4,};


        /* twist denotes the speed component
           pose is the pose component         */    
        /*  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
        // boost::array<double, 36UL> odometry_twist_covariance = {1., 0., 0., 0., 0., 0.,
        //                                                         0., 1., 0., 0., 0., 0.,
        //                                                         0., 0., 1., 0., 0., 0.,
        //                                                         0., 0., 0., 1., 0., 0.,
        //                                                         0., 0., 0., 0., 1., 0.,
        //                                                         0., 0., 0., 0., 0., 1.,};
        
        // /*  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis) */
        // boost::array<double, 36UL> odometry_pose_covariance =  {1., 0., 0., 0., 0., 0.,
        //                                                         0., 1., 0., 0., 0., 0.,
        //                                                         0., 0., 1., 0., 0., 0.,
        //                                                         0., 0., 0., 1., 0., 0.,
        //                                                         0., 0., 0., 0., 1., 0.,
        //                                                         0., 0., 0., 0., 0., 20.};
    public: 
        BaseSensorNode(int, char **);
        ~BaseSensorNode();
        void run();
        
};

BaseSensorNode::~BaseSensorNode(){

}