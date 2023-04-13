#include <SparkFun_TB6612.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

/**********
 * Define *
 *********/
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

/************************
 * Variable Declaration *
 ***********************/
// line up with function names like forward. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

ros::NodeHandle nh;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
  float linear_velocity = cmd_vel_msg.linear.x *200;
  float angular_velocity = cmd_vel_msg.angular.z * 200;

  float left_speed = constrain(linear_velocity - angular_velocity, -255, 255);
  float right_speed = constrain(linear_velocity + angular_velocity, -255, 255);

  motor1.drive(left_speed);
  motor2.drive(right_speed);
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_vel_callback);

void setup()
{
  nh.initNode();
  nh.subscribe(cmd_sub);

}

void loop()
{
  nh.spinOnce();
}
