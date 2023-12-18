/*
 * Version : 0.1
 * Created : 2023.5.14
 * Latest updated : 2023.12.18
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
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/InertiaStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Servo.h>    // servo library for motor control - https://github.com/arduino-libraries/Servo
#include <TimerOne.h> // TimerOne library for threading in arduino mega - https://github.com/PaulStoffregen/TimerOne

// Function prototypes
void publishCmdVelEchoMsg(void);
void waitForSerialLink(bool isConnected);

// callback function prototypes
void messageLEDControlCallback(const std_msgs::Int8 &led_msg);
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel);


#endif // ZZANGDOL_CORE_CONFIG_H_