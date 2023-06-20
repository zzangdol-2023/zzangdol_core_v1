# zzangdol-ai-car firmware

- created : 23-05-14
- last-update : 23-06-04
- maintainer : GeonhaPark | Seunmul <geonhab504@gmail.com>
- contributor : 
- about : zzangdol-ai-car controller code 
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

## Quick Start
### zzangdol_core
- fireware c codes
- platform : arduino mega 2560
- about : core code
- starts with 
```bash
rosrun rosserial_python serial_node.py __name:=zzangdol_core _port:= /dev/ttyUSB0 _baud:=115200
```

### drive_mov_avg
- pure arduino mega codes with moving avg filter, which filters rc controller's input 
