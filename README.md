# zzangdol_core-v1

- created : 23-05-14
- last-update : 24-01-02
- maintainer : GeonhaPark | Seunmul <geonhab504@gmail.com>
- contributor :
- about : zzangdol-ai-car Firmware codes for MCU(Arduino Mega).
  - Version 1.
  - Includes basic communication logic with AP nodes and Car Control logic for zzangdol-ai-car HW
- dependency :
- ros library - https://github.com/frankjoshua/rosserial_arduino_lib
- servo library for motor control - https://github.com/arduino-libraries/Servo
- TimerOne library for threading in arduino mega - https://github.com/PaulStoffregen/TimerOne
- reference code : https://github.com/ROBOTIS-GIT/OpenCR/tree/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_burger/turtlebot3_core

**dependency list must be concluded your Arduino lib packages path**

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
2. **_Arduino : Select Serial Port_** -> Select Port.
3. **_Arduino : Board Config_** -> Select Arudino Mega or your MCU.
4. **_Arduino : Upload_** -> .elf files will be created in the "build" directories. And execution files will be automaticly uploaded to MCU

## 02. Start

### zzangdol_core

- Firmware codes with C++, Arduino Platform
- Platform : Arduino Mega 2560
- Once Build Finished, You can executed rosnodes in the mcu with below codes

### Standard alone Exectuion

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
