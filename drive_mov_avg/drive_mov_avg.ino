#include <Servo.h>
#include <TimerOne.h>
//  20220714

#define Buzzer_Pin 12
#define MOVING_AVG_LEN 5
int led = 13;

// 차량에 무선 수신기로 부터는 PWM 파형이 출력됨. PWM 신호를 받는 핀이 57~59번 핀
byte Steering_PIN = 57;
byte Driving_PIN = 58;
byte Transmission_PIN = 59;

Servo Steering_Servo;     // Servo 클래스로 servo객체 생성
Servo Driving_servo;      // Servo 클래스로 servo객체 생성
Servo Transmission_Servo; // Servo 클래스로 servo객체 생성

unsigned long steering_pulse_value = 90; // 오른쪽 45 ~ 중간 90 ~ 왼쪽140;    // 각도를 조절할 변수 value
int steering = 90;
int steering_sum = steering * MOVING_AVG_LEN; // 이동평균 필터 저장 변수
int steering_center = 90;

unsigned long driving_pulse_value = 90; //   Stop 90 ~ start 96// 각도를 조절할 변수 value
int driving;
int driving_sum = driving * MOVING_AVG_LEN;
int driving_center = 90;

unsigned long transmission_pulse_value = 0;           // pulse신호 입력 변수
int transmission = 135;                               // 2단 35 ~ 3단 90 1단 135
int transmission_sum = transmission * MOVING_AVG_LEN; // 이동평균 필터 저장 변수
int trans_center = 90;

int value;
int data;

void Timer_init()
{
  Timer1.initialize(50000);              // 1000 000us = 1s  //1초마다 타이머 동작
  Timer1.attachInterrupt(Driving_print); // Driver_Oder 함수를 붙임
}

// the setup routine runs once when you press reset:
void setup()
{
  Serial.setTimeout(1);
  pinMode(led, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);

  Steering_Servo.attach(54);     // servo 서보모터 54번 핀에 연결
  Driving_servo.attach(55);      // servo 서보모터 55번 핀에 연결
  Transmission_Servo.attach(56); // servo 서보모터 56번 핀에 연결

  Steering_Servo.write(steering_pulse_value);         // value값의 각도로 회전. ex) value가 90이라면 90도 회전
  Driving_servo.write(driving_pulse_value);           // value값의 각도로 회전. ex) value가 90이라면 90도 회전
  Transmission_Servo.write(transmission_pulse_value); // value값의 각도로 회전. ex) value가 90이라면 90도 회전

  pinMode(Steering_PIN, INPUT);
  pinMode(Driving_PIN, INPUT);
  pinMode(Transmission_PIN, INPUT);

  Serial.begin(115200);

  Buzzer_ON_OFF();
  Buzzer_ON_OFF();
}

void loop()
{
  ////////////////////////Steering//////////////////////////////
  // Steering Control -> 이전 MOVING_AVG_LEN 시퀀스만큼의 이동 평균 필터
  steering_sum -= steering;
  steering_pulse_value = pulseIn(Steering_PIN, HIGH); // 무선 수신기로부터 PWM 파형 받음. 펄스의 길이를 us 단위로 반환
  steering = (int)((steering_pulse_value - 751) / 7.67);

  /* check steering value validity */
  if (steering >= 135)
    steering = 135;
  else if (steering <= 45)
    steering = 45;
  steering_sum += steering;
  steering = 180 - (int)(steering_sum / MOVING_AVG_LEN);
  Serial.print("s_sum: ");
  Serial.print(steering_sum);

  if (steering_center + 2 < steering)
    steering_center = steering - 1;
  if (steering_center - 2 > steering)
    steering_center = steering + 1;

  if (steering_center >= 85 && steering_center <= 90) // 노이즈 때문에 이거 안넣으면 직진에서 덜덜덜 거림.
    steering_center = 90;

  Serial.print("S:");
  Serial.print(steering_center);
  Steering_Servo.write(steering_center);
  steering = 180 - steering;
  //////////////////////////transmission/////////////////////////////////
  // Trasmission Control -> 이전 MOVING_AVG_LEN 시퀀스만큼의 이동 평균 필터
  transmission_pulse_value = pulseIn(Transmission_PIN, HIGH); // 무선 수신기로부터 PWM 파형 받음. 펄스의 길이를 us 단위로 반환
  transmission_sum += (int)((transmission_pulse_value - 751) / 7.67) - transmission;
  transmission = (int)((transmission_sum) / MOVING_AVG_LEN);
  Serial.print("/ t:");
  Serial.print(transmission);
  Serial.print("/ t_sum:");
  Serial.print(transmission_sum);

  /* center value filter */
  transmission += 50;
  if (trans_center + 5 < transmission)
    trans_center = transmission - 1;
  if (trans_center - 5 > transmission)
    trans_center = transmission + 1;
  transmission -= 50;

  /* transmssion moptor control */
  Transmission_Servo.write(trans_center);
  Serial.print("/ T:");
  Serial.print(trans_center);

  /////////////////////////driving////////////////////////////////

  driving_pulse_value = pulseIn(Driving_PIN, HIGH);
  driving = (int)(driving_pulse_value - 790) / 7.67;
  if (driving <= 96 && driving >= 84)
    driving = 90;

  /* check steering value validity */
  if (driving >= 110)
    driving = 110;
  else if (driving <= 70)
    driving = 70;

  Serial.print("/ d_sum:");
  Serial.print(driving_sum);

  Serial.print("/ D:");
  Serial.print(driving);
  Driving_servo.write(driving);

  Serial.print("/ B:");
  Serial.println(analogRead(A7));
}

void Driving_print()
{
  Serial.print("D: ");
  Serial.print(driving);
  Serial.print(", S: ");
  Serial.println(steering);
}
void Buzzer_ON()
{
  // digitalWrite(Buzzer_Pin, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(Buzzer_Pin, HIGH); // turn the LED on (HIGH is the voltage level)
}
void Buzzer_OFF()
{
  // digitalWrite(Buzzer_Pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(Buzzer_Pin, LOW); // turn the LED off by making the voltage LOW
}
void Buzzer_ON_OFF()
{
  Buzzer_ON();
  delay(2); // wait for a second
  Buzzer_OFF();
  // delay(1);                       // wait for a second
}
