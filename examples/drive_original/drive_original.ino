#include <Servo.h>
#include <TimerOne.h>
//  20220714

#define Buzzer_Pin 12

int led = 13;

byte Steering_PIN = 57;
byte Driving_PIN = 58;
byte Transmission_PIN = 59;

Servo Steering_servo;      //Servo 클래스로 servo객체 생성
Servo Driving_servo;       //Servo 클래스로 servo객체 생성
Servo Transmission_servo;  //Servo 클래스로 servo객체 생성

int Steering_value = 90;       //오른쪽 45 ~ 중간 90 ~ 왼쪽140;    // 각도를 조절할 변수 value
int Driving_value = 90;        //   Stop 90 ~ start 96// 각도를 조절할 변수 value
int Transmission_value = 135;  // 2단 35 ~ 3단 90 1단 135 // 각도를 조절할 변수 value


int Steering;
int Driving;
int Transmission;
int trans_center = 90;


int value;
int data;

void Timer_init() {
  Timer1.initialize(50000);               //1000 000us = 1s  //1초마다 타이머 동작
  Timer1.attachInterrupt(Driving_print);  //Driver_Oder 함수를 붙임
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.setTimeout(1);
  pinMode(led, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);

  Steering_servo.attach(54);      //servo 서보모터 54번 핀에 연결
  Driving_servo.attach(55);       //servo 서보모터 55번 핀에 연결
  Transmission_servo.attach(56);  //servo 서보모터 56번 핀에 연결

  Steering_servo.write(Steering_value);          //value값의 각도로 회전. ex) value가 90이라면 90도 회전
  Driving_servo.write(Driving_value);            //value값의 각도로 회전. ex) value가 90이라면 90도 회전
  Transmission_servo.write(Transmission_value);  //value값의 각도로 회전. ex) value가 90이라면 90도 회전

  pinMode(Steering_PIN, INPUT);
  pinMode(Driving_PIN, INPUT);
  pinMode(Transmission_PIN, INPUT);



  Serial.begin(115200);

  Buzzer_ON_OFF();
  Buzzer_ON_OFF();
}

void loop() {
  ////////////////////////Steering//////////////////////////////
  for (value = 0; value < 3; value++) {
    Steering_value = pulseIn(Steering_PIN, HIGH);
    Steering = (Steering_value - 751) / 7.67;
    data += Steering;
  }
  Steering = data / 3;
  data = 0;

  Steering = 180 - Steering;
  if(Steering <= 100 && Steering >= 80)
    Steering = 90;
  Steering_servo.write(Steering);
  Serial.print("S:");
  Serial.print(Steering);

  //////////////////////////transmission/////////////////////////////////
  for (value = 0; value < 3; value++) {
    Transmission_value = pulseIn(Transmission_PIN, HIGH);
    Transmission = (Transmission_value - 751) / 7.67;
    data += Transmission;
  }
  Transmission = data / 3;
  data = 0;

  Transmission += 50;

  if(trans_center + 5 < Transmission)
    trans_center = Transmission - 1;

  if(trans_center - 5 > Transmission)
    trans_center = Transmission + 1;
  
  Transmission_servo.write(trans_center);
  Serial.print("/ T:");
  Serial.print(trans_center);

  


  /////////////////////////driving////////////////////////////////

  Driving_value = pulseIn(Driving_PIN, HIGH);
  Driving = (Driving_value - 790) / 7.67;

  if (Driving <= 96 && Driving >= 84)
    Driving = 90;
  Driving_servo.write(Driving);
  Serial.print("/ D:");
  Serial.print(Driving);

  Serial.print("/ B:");
  Serial.println(analogRead(A7));
}


void Driving_print() {
  Serial.print("D: ");
  Serial.print(Driving);
  Serial.print(", S: ");
  Serial.println(Steering);
}
void Buzzer_ON() {
  //digitalWrite(Buzzer_Pin, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(Buzzer_Pin, HIGH);  // turn the LED on (HIGH is the voltage level)
}
void Buzzer_OFF() {
  //digitalWrite(Buzzer_Pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(Buzzer_Pin, LOW);  // turn the LED off by making the voltage LOW
}
void Buzzer_ON_OFF() {
  Buzzer_ON();
  delay(2);  // wait for a second
  Buzzer_OFF();
  //delay(1);                       // wait for a second
}
