#include <Adafruit_MPU6050.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

/**********
 * Define *
 *********/
#define Delay 25

/************************
 * Variable Declaration *
 ***********************/
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data_raw", &imu_msg);

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel, *mpu_gyro;

void setup() {
  if (!mpu.begin())  while (1);
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  mpu_gyro->printSensorDetails();

  nh.initNode();
  nh.advertise(imu_pub);
}

void loop() {
  //Get a new normalized sensor event
  sensors_event_t accel;
  sensors_event_t gyro;
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  
  imu_msg.header.stamp = nh.now();
  
  // Set the orientation quaternion to be all zeros, madgwick filter to be implemented in the ROS side
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;

  // Set the angular velocity values in the message
  imu_msg.angular_velocity.x = gyro.gyro.x;
  imu_msg.angular_velocity.y = gyro.gyro.y;
  imu_msg.angular_velocity.z = gyro.gyro.z;

  // Set the linear acceleration values in the message
  imu_msg.linear_acceleration.x = accel.acceleration.x;
  imu_msg.linear_acceleration.y = accel.acceleration.y;
  imu_msg.linear_acceleration.z = accel.acceleration.z;

  // Publish the IMU message
  imu_pub.publish(&imu_msg);
    nh.spinOnce();

  // Wait for a short period of time before publishing again
  delay(Delay);
}
