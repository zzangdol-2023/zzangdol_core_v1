/*
 * Version : 0.1
 * Created : 2023.5.14
 * Latest updated : 2023.5.20
 * Maintainer : GeonhaPark <geonhab504@gmail.com>,
 *              GeoChoi,
 *              GyucheolJung,
 *              JungcheolBaek
 * About : zzangdol car opencr wrapper to arduino mega
 * Reference : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_waffle/turtlebot3_core/turtlebot3_core_config.h
 */

#ifndef ZZANGDOL_CORE_CONFIG_H_
#define ZZANGDOL_CORE_CONFIG_H_

#include <ros.h> // ros library - https://github.com/frankjoshua/rosserial_arduino_lib
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/InertiaStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Servo.h>    // servo library for motor control - https://github.com/arduino-libraries/Servo
#include <TimerOne.h> // TimerOne library for threading in arduino mega - https://github.com/PaulStoffregen/TimerOne

#define CONTROL_MOTOR_SPEED_FREQUENCY 30        // hz
#define CONTROL_MOTOR_TIMEOUT 500               // ms
#define IMU_PUBLISH_FREQUENCY 200               // hz
#define CMD_VEL_PUBLISH_FREQUENCY 30            // hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY 30  // hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY 1 // hz
#define DEBUG_LOG_FREQUENCY 10                  // hz

#define WHEEL_NUM 2
#define LINEAR 0
#define ANGULAR 1

/* PIN Define */
#define BUZZER_PIN 12 // BUZZER PIN
#define STEERING_SERVO_OUT_PIN 54
#define DRIVING_SERVO_OUT_PIN 55
#define TRANSMISSION_SERVO_OUT_PIN 56
#define STEERING_RC_IN_PIN 57 // 차량에 무선 수신기로부터는 PWM 파형이 출력됨. PWM 신호를 받는 핀이 57~59번 핀
#define DRIVING_RC_IN_PIN 58
#define TRANSMISSION_RC_IN_PIN 59

/* MAX VALUES */
#define MIN_LINEAR_VELOCITY -45.0
#define MAX_LINEAR_VELOCITY +45.0
#define MIN_ANGULAR_VELOCITY -45.0
#define MAX_ANGULAR_VELOCITY +45.0

/* VARIABLES */
#define MOVING_AVG_LEN 5

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
 * SoftwareTimer of ZZANGDOL
 *******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
 * Declaration for controllers
 *******************************************************************************/
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc[WHEEL_NUM] = {0.0, 0.0};

// Function prototypes
void publishCmdVelEchoMsg(void);
void waitForSerialLink(bool isConnected);

// callback function prototypes
void messageLEDControlCallback(const std_msgs::Int8 &led_msg);
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel);

/*******************************************************************************
 * Subscribers
 *******************************************************************************/
ros::Subscriber<std_msgs::Int8> led_sub("led", messageLEDControlCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel_converted", commandVelocityCallback);
// ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
 * Publishers
 *******************************************************************************/

// cmd_vel_echo functions
geometry_msgs::Twist cmd_vel_echo;
ros::Publisher cmd_vel_echo_pub("cmd_vel_echo", &cmd_vel_echo);

/*******************************************************************************
 * Transform Broadcaster
 *******************************************************************************/
// TF of zzangdol-car
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

#endif // ZZANGDOL_CORE_CONFIG_H_