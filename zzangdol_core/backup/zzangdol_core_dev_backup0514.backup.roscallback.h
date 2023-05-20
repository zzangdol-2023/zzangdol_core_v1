// ros library
// https://github.com/frankjoshua/rosserial_arduino_lib
#include <ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/InertiaStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
 * SoftwareTimer of Turtlebot3
 *******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
 * Declaration for controllers
 *******************************************************************************/

float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_button[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};

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

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

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