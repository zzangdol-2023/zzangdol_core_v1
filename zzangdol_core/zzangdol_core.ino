/*
 * Version : 0.1
 * Created : 2023.5.14
 * Latest updated : 2023.6.3
 * Maintainer : GeonhaPark <geonhab504@gmail.com>,
 *              GeoChoi,
 *              GyucheolJung,
 *              JungcheolBaek,
 *              JungheonSong
 * About : zzangdol car firmware | opencr wrapper to arduino mega
 * Reference : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_waffle/turtlebot3_core/turtlebot3_core.ino
 */

#include "zzangdol_core_config.h"

Servo Steering_Servo;     // Servo 클래스로 servo객체 생성
Servo Driving_Servo;      // Servo 클래스로 servo객체 생성
Servo Transmission_Servo; // Servo 클래스로 servo객체 생성

unsigned long steering_pulse_value = 0;       // pulse신호 입력 변수
int steering = 90;                            // 오른쪽 45 ~ 중간 90 ~ 왼쪽140
int steering_sum = steering * MOVING_AVG_LEN; // 이동평균 필터 저장 변수
int steering_center = 90;

unsigned long driving_pulse_value = 0;      // pulse신호 입력 변수
int driving = 90;                           // Stop 90 ~ start 96
int driving_sum = driving * MOVING_AVG_LEN; // 이동평균 필터 저장 변수

unsigned long transmission_pulse_value = 0;           // pulse신호 입력 변수
int transmission = 135;                               // 2단 35 ~ 3단 90 1단 135
int transmission_sum = transmission * MOVING_AVG_LEN; // 이동평균 필터 저장 변수
int trans_center = 90;

int value_cnt = 0;    // 반복문 cnt 변수
int data_tmp = 0;     // 임시변수
bool mode_rc = false; // false 일 경우 자율주행모드, true일 경우 rc카 모드

int back_driving_flag = 0;     // 후진 flag 변수
int back_driving_flag_cnt = 0; // 후진 flag 변수

uint32_t t = 0; // 전역 시간 변수

//--------* Arduino main Functions : Setup & loop *--------//
// the setup routine runs once when you press reset:
void setup()
{
    /* ros node setup*/
    nh.initNode();
    nh.getHardware()->setBaud(115200);

    /* subscriber node setup */
    nh.subscribe(led_sub);
    nh.subscribe(cmd_vel_sub);

    /* publisher node setup */
    nh.advertise(cmd_vel_echo_pub);

    // tf_broadcaster.init(nh);

    /* set pin mode */
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    Serial.setTimeout(1);

    /* setup controller(RC) input pins */
    pinMode(STEERING_RC_IN_PIN, INPUT);
    pinMode(DRIVING_RC_IN_PIN, INPUT);
    pinMode(TRANSMISSION_RC_IN_PIN, INPUT);

    /* setup control pins */
    Steering_Servo.attach(STEERING_SERVO_OUT_PIN);         // servo 서보모터 54번 핀에 연결
    Driving_Servo.attach(DRIVING_SERVO_OUT_PIN);           // servo 서보모터 55번 핀에 연결
    Transmission_Servo.attach(TRANSMISSION_SERVO_OUT_PIN); // servo 서보모터 56번 핀에 연결
    Steering_Servo.write(steering_center);                 // value값의 각도로 회전. ex) value가 90이라면 90도 회전
    Driving_Servo.write(driving);                          // value값만큼 회전. 속도조절용.
    Transmission_Servo.write(transmission);                // value값의 각도로 회전. ex) value가 90이라면 90도 회전

    /* buzzer notifies setup complete */
    Buzzer_ON_OFF();
    Buzzer_ON_OFF();
}
void loop()
{
    t = millis();
    updateTime(); // ros time update

    /*
     * motor contorol section
     */
    if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
    {
        updateGoalVelocity(); // 특정 주기마다 goal velocity 업데이트
        if ((t - tTime[6]) > CONTROL_MOTOR_TIMEOUT)
        {
            // 일정시간 동안 입력이 없을 시 0의 속도로 구동
            goal_velocity_from_cmd[LINEAR] = 0;
            goal_velocity_from_cmd[ANGULAR] = 0;
            goal_velocity[LINEAR] = zero_velocity[LINEAR];
            goal_velocity[ANGULAR] = zero_velocity[ANGULAR];
            controlMotor(zero_velocity);
        }
        else
        {
            // timeout이 아닐경우 goal velocity로 구동
            controlMotor(goal_velocity);
        }
        tTime[0] = t;
    }

    /* publish cmd_vel_echo msg - 특정 주기마다 echo cmd_vel 데이터 퍼블리시 */
    if ((t - tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
    {
        publishCmdVelEchoMsg();
        tTime[1] = t;
    }

    /* RC mode check는 하단에서 한번에 해야 time이 꼬이지 않음. */
    getDataFromRCController(); // RC컨트롤러로부터 제어 정보 지속적으로 업데이트
    if (mode_rc)
    {
        tTime[6] = millis();
    }

    // Call all the callbacks waiting to be called at that point in time
    nh.spinOnce();

    // Wait the serial link time to process
    waitForSerialLink(nh.connected());
}

/*******************************************************************************
 * Publisher functions
 *******************************************************************************/
void publishCmdVelEchoMsg()
{
    // if ((t - tTime[6]) > CONTROL_MOTOR_TIMEOUT) // motor timeout zero value echo
    // {
    //     cmd_vel_echo.linear.x = zero_velocity[LINEAR];
    //     cmd_vel_echo.angular.z = zero_velocity[ANGULAR];
    // }
    // else
    // {
    // }
    cmd_vel_echo.linear.x = round(goal_velocity[LINEAR]);
    cmd_vel_echo.angular.z = round(goal_velocity[ANGULAR]);
    cmd_vel_echo_pub.publish(&cmd_vel_echo);
}

/*******************************************************************************
 * Callback functions
 *******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    /* change data which will be published to ros core node */
    goal_velocity_from_cmd[LINEAR] = cmd_vel.linear.x;   // 직선 주행 데이터 받기 및 cmd velocity 업데이트
    goal_velocity_from_cmd[ANGULAR] = cmd_vel.angular.z; // 각속도 주행 데이터 받기 및 cmd velocity 업데이트

    /* 최대 최소 범위 안으로 입력받게끔 설정 */
    goal_velocity_from_cmd[LINEAR] = constrain(goal_velocity_from_cmd[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    tTime[6] = millis();
}

void messageLEDControlCallback(const std_msgs::Int8 &led_msg)
{
    if (led_msg.data == 1)
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    else
        digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
}

/*******************************************************************************
 * get controller data from RC Controller
 *******************************************************************************/
void getDataFromRCController()
{
    /* Trasmission Control -> 이전 MOVING_AVG_LEN 시퀀스만큼의 이동 평균 필터 */
    /******************************************************************/
    transmission_pulse_value = pulseIn(TRANSMISSION_RC_IN_PIN, HIGH); // 무선 수신기로부터 PWM 파형 받음. 펄스의 길이를 us 단위로 반환
    transmission_sum += (int)((transmission_pulse_value - 751) / 7.67) - transmission;
    transmission = (int)((transmission_sum) / MOVING_AVG_LEN);

    /* center value filter */
    transmission += 50;
    if (trans_center + 5 < transmission)
        trans_center = transmission - 1;
    if (trans_center - 5 > transmission)
        trans_center = transmission + 1;

    /* transmission 신호가 120보다 작을 경우 RC 모드로, 클 경우 자율주행 모드로*/
    transmission < 120 ? mode_rc = true : mode_rc = false;
    transmission -= 50;

    /* transmssion motor control */
    Transmission_Servo.write(135);

    /* Steering Control -> 이전 MOVING_AVG_LEN 시퀀스만큼의 이동 평균 필터 */
    /******************************************************************/
    steering_sum -= steering;
    steering_pulse_value = pulseIn(STEERING_RC_IN_PIN, HIGH); // 무선 수신기로부터 PWM 파형 받음. 펄스의 길이를 us 단위로 반환
    steering = (int)((steering_pulse_value - 751) / 7.67);

    /* check steering value validity */
    if (steering >= 110)
        steering = 110;
    else if (steering <= 70)
        steering = 70;
    steering_sum += steering;
    steering = 180 - (int)(steering_sum / MOVING_AVG_LEN);

    /* center value filter */
    if (steering_center + 1 < steering)
        steering_center = steering - 1;
    if (steering_center - 1 > steering)
        steering_center = steering + 1;
    if (steering_center >= 84 && steering_center <= 96) // 노이즈 때문에 이거 안넣으면 직진에서 덜덜덜 거림.
        steering_center = 90;

    steering = 180 - steering; // rollback steering value

    /* Driving Control -> 이전 MOVING_AVG_LEN 시퀀스만큼의 이동 평균 필터 */
    /******************************************************************/
    driving_pulse_value = pulseIn(DRIVING_RC_IN_PIN, HIGH);
    driving = (int)((driving_pulse_value - 790) / 7.67);
    if (driving <= 96 && driving >= 84)
        driving = 90;

    /* check steering value validity */
    if (driving >= 99)
    {
        driving = 99;
    }
    else if (driving <= 78)
        driving = 78;

    /* rc컨트롤러로 부터 제어신호 값 업데이트 -> 정규화를 위해 -90 */
    goal_velocity_from_rc[LINEAR] = (float)driving - 90.0;
    goal_velocity_from_rc[ANGULAR] = (float)steering_center - 90.0;
}
/*******************************************************************************
 * controlMotor function
 *******************************************************************************/
void controlMotor(float velocity[])
{
    /* motor control data update */
    Driving_Servo.write((int)round(velocity[LINEAR]) + 90);
    Steering_Servo.write((int)round(velocity[ANGULAR]) + 90);
}

/*******************************************************************************
 * Wait for Serial Link
 *******************************************************************************/
void waitForSerialLink(bool isConnected)
{
    static bool wait_flag = false;

    if (isConnected)
    {
        if (wait_flag == false)
        {
            delay(10);
            wait_flag = true;
        }
    }
    else
    {
        wait_flag = false;
    }
}

/*******************************************************************************
 * Update the base time for interpolation
 *******************************************************************************/
void updateTime()
{
    current_offset = millis();
    current_time = nh.now();
}

/*******************************************************************************
 * Update Goal Velocity
 *******************************************************************************/
void updateGoalVelocity(void)
{
    if (mode_rc) // rc 모드일 경우 goal velocity를 RC컨트롤러 값으로 업데이트
    {
        goal_velocity[LINEAR] = goal_velocity_from_rc[LINEAR];
        goal_velocity[ANGULAR] = goal_velocity_from_rc[ANGULAR];
    }
    else // 자율주행 모드일 경우 cmd 데이터로 부터 goal velocity 업데이트
    {
        goal_velocity[LINEAR] = goal_velocity_from_cmd[LINEAR];
        goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
    }
}

/*******************************************************************************
 * Huins-AI-Car Bundled Function
 *******************************************************************************/
void Buzzer_ON()
{
    digitalWrite(BUZZER_PIN, HIGH); // turn the LED on (HIGH is the voltage level)
}
void Buzzer_OFF()
{
    digitalWrite(BUZZER_PIN, LOW); // turn the LED off by making the voltage LOW
}
void Buzzer_ON_OFF()
{
    Buzzer_ON();
    delay(2); // wait for a second
    Buzzer_OFF();
}