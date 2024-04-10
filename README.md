# zzangdol_core-v1

- Created: 23-05-14
- Last update: 24-0-02
- Maintainer: GeonhaPark | Seunmul <geonhab504@gmail.com>
- Contributor:
- About:
  + Zzangdol-ai-car Firmware codes for MCU (Arduino Mega).
  + Version 1.
  + Includes communication logic between MPU nodes and MCU -- car control logic for zzangdol-ai-car HW
- Dependency:
  + ROS library - https://github.com/frankjoshua/rosserial_arduino_lib
  + Servo library for motor control - https://github.com/arduino-libraries/Servo
  + TimerOne library for supporting multithread in Arduino Mega - https://github.com/PaulStoffregen/TimerOne
- Reference codes
  + https://github.com/ROBOTIS-GIT/OpenCR/tree/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core

**Dependency codes must be included in your Arduino library package PATH**

- Linux : /home/$USER/Arduino/libraries  
  Arduino  
  └── libraries  
   ├── MPU9250  
   ├── ros_lib  
   ├── Servo  
   └── TimerOne

## 01. Build

### Environment

vscode with Arduino Extension : https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino (ver : 0.6.0)

### How to Build

0. Install Extension and Move to Target directory.
1. execute with Ctrl+P, Search Arduino, and Click below menu.
2. **_Arduino: Select Serial Port_** -> Select Port.
3. **_Arduino: Board Config_** -> Select Arudino Mega or your MCU.
4. **_Arduino: Upload_** -> .elf files will be created in the "build" directories. Execution files will be automatically uploaded to MCU

## 02. Start

### zzangdol_core

- Firmware codes with C++, Arduino Platform
- Platform: Arduino Mega 2560
- Once the Build is Finished, You can execute ROS nodes in the MCU with the below codes

### Standard alone Execution

```bash
rosrun rosserial_python serial_node.py __name:=zzangdol_core _port:= /dev/ttyUSB0 _baud:=115200
```

### With roslaunch commands

```bash
# requires zzangdol_bringup launch files & dependecies : https://github.com/zzangdol-2023/zzangdol_bringup
roslaunch zzangdol_bringup zzangdol_core.launch usb_config:=true
```

## 03. Directories

### zzangdol_core

- zzangdol-ai-car Firmware codes for MCU(Arduino Mega)

### drive_mov_avg

- Pure firmware codes with moving-avg-filter, which filters extern-RC-controller's input

### examples

- Backup & example codes for Arudino Mega & ROS platform
