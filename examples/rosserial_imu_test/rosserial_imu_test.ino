/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

/*
 * version : 0.1
 * created : 2023.5.14
 * latest updated : 2023.5.14
 * overview : test codes of rosserial & mpu9250
 * maintainer : GeonhaPark <geonhab504@gmail.com>
 * about : opencr wrapper to arduino mega
 */

/*
 * MPU9250 <-> UNO
 *   VCC       5V
 *   GND       GND
 *   SCL       A5
 *   SDA       A4
 *   INT       2
 * --------------
 * MPU9250 <-> MEGA
 *   VCC       5V
 *   GND       GND
 *   SCL       SCL(21)
 *   SDA       SDA(20)
 *   INT       2
 */
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "MPU9250.h"

/*
 * Library from : https://github.com/hideakitai/MPU9250
 * check the sample codes in the github or library => examples
 */

// cIMU imu;
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 mpu;

void setup()
{
    /*init ros node handler*/
    nh.initNode();
    nh.advertise(imu_pub);
    tfbroadcaster.init(nh);

    /*setup imu sensor*/
    // imu.begin();
    // Serial.begin(115200);
    Wire.begin();
    if (!mpu.setup(0x68))
    { // change to your own address
        while (1)
        {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    mpu.calibrateAccelGyro();
    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(5000);
    mpu.calibrateMag();
    // print_calibration();
    mpu.verbose(false);
}

void loop()
{
    static uint32_t pre_time;

    // imu.update();
    if (mpu.update())
    {
        if (millis() - pre_time >= 50)
        {
            pre_time = millis();

            imu_msg.header.stamp = nh.now();
            imu_msg.header.frame_id = "imu_link";

            // imu_msg.angular_velocity.x = imu.gyroData[0];
            imu_msg.angular_velocity.x = mpu.getGyroX();
            // imu_msg.angular_velocity.y = imu.gyroData[1];
            imu_msg.angular_velocity.y = mpu.getGyroY();
            // imu_msg.angular_velocity.z = imu.gyroData[2];
            imu_msg.angular_velocity.z = mpu.getGyroZ();

            imu_msg.angular_velocity_covariance[0] = 0.02;
            imu_msg.angular_velocity_covariance[1] = 0;
            imu_msg.angular_velocity_covariance[2] = 0;
            imu_msg.angular_velocity_covariance[3] = 0;
            imu_msg.angular_velocity_covariance[4] = 0.02;
            imu_msg.angular_velocity_covariance[5] = 0;
            imu_msg.angular_velocity_covariance[6] = 0;
            imu_msg.angular_velocity_covariance[7] = 0;
            imu_msg.angular_velocity_covariance[8] = 0.02;

            /* get linear acceration data from mpu9250 and set msgs*/
            // imu_msg.linear_acceleration.x = imu.accData[0];
            imu_msg.linear_acceleration.x = mpu.getLinearAccX();
            // imu_msg.linear_acceleration.y = imu.accData[1];
            imu_msg.linear_acceleration.x = mpu.getLinearAccY();
            // imu_msg.linear_acceleration.z = imu.accData[2];
            imu_msg.linear_acceleration.x = mpu.getLinearAccZ();
            imu_msg.linear_acceleration_covariance[0] = 0.04;
            imu_msg.linear_acceleration_covariance[1] = 0;
            imu_msg.linear_acceleration_covariance[2] = 0;
            imu_msg.linear_acceleration_covariance[3] = 0;
            imu_msg.linear_acceleration_covariance[4] = 0.04;
            imu_msg.linear_acceleration_covariance[5] = 0;
            imu_msg.linear_acceleration_covariance[6] = 0;
            imu_msg.linear_acceleration_covariance[7] = 0;
            imu_msg.linear_acceleration_covariance[8] = 0.04;

            /* get quaternion data from mpu9250 and set msgs*/
            // imu_msg.orientation.w = imu.quat[0];
            imu_msg.orientation.w = mpu.getQuaternionW();
            // imu_msg.orientation.x = imu.quat[1];
            imu_msg.orientation.x = mpu.getQuaternionX();
            // imu_msg.orientation.y = imu.quat[2];
            imu_msg.orientation.y = mpu.getQuaternionY();
            // imu_msg.orientation.z = imu.quat[2];
            imu_msg.orientation.z = mpu.getQuaternionZ();

            imu_msg.orientation_covariance[0] = 0.0025;
            imu_msg.orientation_covariance[1] = 0;
            imu_msg.orientation_covariance[2] = 0;
            imu_msg.orientation_covariance[3] = 0;
            imu_msg.orientation_covariance[4] = 0.0025;
            imu_msg.orientation_covariance[5] = 0;
            imu_msg.orientation_covariance[6] = 0;
            imu_msg.orientation_covariance[7] = 0;
            imu_msg.orientation_covariance[8] = 0.0025;

            imu_pub.publish(&imu_msg);

            tfs_msg.header.stamp = nh.now();
            tfs_msg.header.frame_id = "base_link";
            tfs_msg.child_frame_id = "imu_link";
            // tfs_msg.transform.rotation.w = imu.quat[0];
            tfs_msg.transform.rotation.w = mpu.getQuaternionW();
            // tfs_msg.transform.rotation.x = imu.quat[1];
            tfs_msg.transform.rotation.x = mpu.getQuaternionX();
            // tfs_msg.transform.rotation.y = imu.quat[2];
            tfs_msg.transform.rotation.y = mpu.getQuaternionY();
            // tfs_msg.transform.rotation.z = imu.quat[3];
            tfs_msg.transform.rotation.z = mpu.getQuaternionZ();

            tfs_msg.transform.translation.x = 0.0;
            tfs_msg.transform.translation.y = 0.0;
            tfs_msg.transform.translation.z = 0.0;

            tfbroadcaster.sendTransform(tfs_msg);
        }
    }
    nh.spinOnce();
}

void print_roll_pitch_yaw()
{
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_calibration()
{
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
