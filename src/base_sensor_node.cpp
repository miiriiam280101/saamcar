#include "base_sensor_node.hpp"

int main(int argc, char ** argv){
    BaseSensorNode base_sensor_node(argc, argv);
    base_sensor_node.run();
}

BaseSensorNode::BaseSensorNode(int argc, char ** argv){

    // Initialize ROS Node and get publisher

    ros::init(argc, argv, "base_sensor_node");

    ros::NodeHandle n;
    
    imu_publisher          = n.advertise<sensor_msgs::Imu>("base_sensor/imu", 100);
    wheel_data_publisher   = n.advertise<saamcar::Encoder>("base_sensor/wheel_data", 100);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Initialize Serial Device
      if(rear_imu_wheel_device.init(ARDUINO_REAR_IMU_WHEELENC, 
            SERIAL_DEVICE_PREFIX, NUM_ARDUINO))
    {
        ROS_INFO("Connected to arduino on port: %d\t ID: %d\t Software version: %d",
            rear_imu_wheel_device.get_port_num(), rear_imu_wheel_device.get_id(), rear_imu_wheel_device.get_software_version());
    }
    else
    {
        ROS_ERROR("Could not find an arduino with correct id: %d ", ARDUINO_REAR_IMU_WHEELENC);
    }

}

void BaseSensorNode::run(){
    while(ros::ok()){
        poll();
        ros::spinOnce();
    }
}

void BaseSensorNode::poll(){

    /* ADAS CODE */
    std::vector<uint8_t> frame;

    if (rear_imu_wheel_device.get_next_frame(frame))
    {
        tArduinoHeader header;
        memcpy(&header, frame.data(), sizeof(tArduinoHeader));
        tDataUnion data;
        memcpy(&data, frame.data() + sizeof(tArduinoHeader), header.ui8DataLength);
        const SENSOR_ID id = static_cast<SENSOR_ID>(header.ui8ID);
        const uint32_t timestamp = header.ui32Timestamp;

        switch (id)
        {
        case ID_ARD_SENSOR_INFO:
            // ROS_INFO("Info frame received.\tID: %d\tSoftware version: %d",
            //                     data.info.ui8ArduinoAddress, data.info.ui16ArduinoVersion);
            break;
        case ID_ARD_SENS_ERROR:
            // ROS_INFO(cString::Format("Error frame received from port: %s%d", 
            //                     m_serialDevicePrefix.c_str(), m_serialDevice.get_id()));
            ROS_INFO("ERROR");
            break;

        case ID_ARD_SENS_IMU:
            processIMU(timestamp, data);
            break;
        
        case ID_ARD_SENS_WHEEL_LEFT:
        case ID_ARD_SENS_WHEEL_RIGHT:
            processENC(timestamp, data, id);
            break;
        default:
            // ROS_INFO("## Got frame from Arduino: %d \n\t Frame ID: %d \n\t Frame Data: %d",
            //             rear_imu_wheel_device.get_id(), id, data);
            break;
        }
    }

}

void BaseSensorNode::processIMU(uint32_t timestamp, tDataUnion data){
    // TODO: Magnetometer Message (absolute readings)
    sensor_msgs::Imu imu_msg;

    current_time = ros::Time::now();

    // f32Ax => Accelerometer
    imu_msg.linear_acceleration.x = data.imu.f32ax * 9.81;
    imu_msg.linear_acceleration.y = data.imu.f32ay * 9.81;
    imu_msg.linear_acceleration.z = data.imu.f32az * 9.81;

    // f32Gx => Gyro
    imu_msg.angular_velocity.x    = data.imu.f32gx * M_PI / 180.;
    imu_msg.angular_velocity.y    = data.imu.f32gy * M_PI / 180.;
    imu_msg.angular_velocity.z    = data.imu.f32gz * M_PI / 180.;


    imu_msg.orientation.w         = 0;
    imu_msg.orientation.x         = 0;
    imu_msg.orientation.y         = 0;
    imu_msg.orientation.z         = 0;


    /* fill imu_msg covariance, */
    /* Following the message imu_msg (ros docs) definition: 
        -1 should be put into the covariance matrix of unavailable data. 
        An zero matrix describes just the unavailability of covariance data 
        none_array(zero matrix) and undefined_array(-1 matrix) defined in header file */


    imu_msg.angular_velocity_covariance =  imu_angular_covariance;
    imu_msg.linear_acceleration_covariance = none_array;
    imu_msg.orientation_covariance = undefined_array;
    
    /* Fill in Header Info */
    imu_msg.header.frame_id = "imu";
    imu_msg.header.stamp = current_time;

    /* PUBLISH IMU DATA */
    imu_publisher.publish(imu_msg);
 }

 void BaseSensorNode::processENC(uint32_t timestamp, tDataUnion data, SENSOR_ID id){
    std_msgs::Int32 data_msg;
    std_msgs::Bool  dir_msg;

    data_msg.data = data.wheel.ui32WheelTach;
    dir_msg.data = data.wheel.i8WheelDir;
     
    switch (id)
    {
    case ID_ARD_SENS_WHEEL_LEFT:
        /* Save Left Wheel Frame */
        left_wheel_timestamp = timestamp;
        left_wheel_data = data.wheel.ui32WheelTach;
        left_wheel_dir = (data.wheel.i8WheelDir != 0);
        break;
    case ID_ARD_SENS_WHEEL_RIGHT:
        /* Save Right Wheel Frame */
        right_wheel_timestamp = timestamp;
        right_wheel_data = data.wheel.ui32WheelTach;
        right_wheel_dir = (data.wheel.i8WheelDir != 0);
        break;

    default:
        break;
    }

    /* If recieved right frame is close to the left frame an Odometry frame can be crafted */
    if (abs(right_wheel_timestamp - left_wheel_timestamp) < wheel_packet_timeout){
        valid_wheel_packet = true;
        ROS_INFO("lost an encoder packet: time: %d", abs(right_wheel_timestamp - left_wheel_timestamp));
    }else{
        ROS_INFO("lost an encoder packet: time: %d", abs(right_wheel_timestamp - left_wheel_timestamp));
        valid_wheel_packet = false;
    }

    // often first few frames are garbage; throwing away 10 packets
    if (init_wheel_data & valid_wheel_packet){
        init_garbage_counter++;
        last_left_wheel_data = left_wheel_data;
        last_right_wheel_data = right_wheel_data;

        if(init_garbage_counter > 10) {
            init_wheel_data = false;
        }
    }

    /* Send Odometry Message */
    if(valid_wheel_packet & !init_wheel_data){
        processOdom();
    }

 }

 void BaseSensorNode::processOdom(){
    saamcar::Encoder encoder_message;

    encoder_message.left_wheel_data = left_wheel_data;
    encoder_message.left_wheel_dir = left_wheel_dir;
    encoder_message.right_wheel_data = right_wheel_data;
    encoder_message.right_wheel_dir = right_wheel_dir;

    encoder_message.header.frame_id="wheel_frame";
    encoder_message.header.stamp = ros::Time::now();

    wheel_data_publisher.publish(encoder_message);
 }
