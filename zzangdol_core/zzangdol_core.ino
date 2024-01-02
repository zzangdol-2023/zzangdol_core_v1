/*
 * Version : 0.3.0
 * Created : 2023.5.14
 * Latest updated : 2023.12.15
 * Maintainer : GeonhaPark <geonhab504@gmail.com>,
 *              GeoChoi,
 *              GyucheolJung,
 *              JungcheolBaek,
 *              JungheonSong
 * About : zzangdol car firmware | opencr wrapper to arduino mega
 * Reference : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3/examples/turtlebot3_waffle/turtlebot3_core/turtlebot3_core.ino
 */

#include "zzangdol_core_config.h"
#define CONTROL_MOTOR_SPEED_FREQUENCY 20 // hz
#define CONTROL_MOTOR_TIMEOUT 750        // ms
#define CMD_VEL_MSG_PUBLISH_FREQUENCY 30 // hz
#define DEBUG_LOG_FREQUENCY 10           // hz

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
#define BLUE_SWITCH 2
#define YELLOW_SWITCH 3
#define WEIGHT_COUNT 7

/* MAX VALUES */
#define MIN_LINEAR_VELOCITY -30.0
#define MAX_LINEAR_VELOCITY +30.0
#define MIN_ANGULAR_VELOCITY -30.0
#define MAX_ANGULAR_VELOCITY +30.0
#define MAX_RC_VELOCITY 102.0
#define MIN_RC_VELOCITY 75.0

/* VARIABLES */
#define MOVING_AVG_LEN 5

/* DEBUG FLAG */
#define SERIAL_DEBUG 0

Servo Steering_Servo;     // Servo 클래스로 servo객체 생성
Servo Driving_Servo;      // Servo 클래스로 servo객체 생성
Servo Transmission_Servo; // Servo 클래스로 servo객체 생성

bool mode_rc = false; // false 일 경우 자율주행모드, true일 경우 rc카 모드

unsigned long steering_pulse_value = 0;       // pulse신호 입력 변수
int steering = 90;                            // 오른쪽 45 ~ 중간 90 ~ 왼쪽140
int steering_sum = steering * MOVING_AVG_LEN; // 이동평균 필터 저장 변수
int steering_center = 90;

unsigned long driving_pulse_value = 0; // pulse신호 입력 변수
int driving = 90;                      // Stop 90 ~ start 96
// int driving_sum = driving * MOVING_AVG_LEN; // 이동평균 필터 저장 변수

unsigned long transmission_pulse_value = 0;           // pulse신호 입력 변수
int transmission = 135;                               // 2단 35 ~ 3단 90 1단 135
int transmission_sum = transmission * MOVING_AVG_LEN; // 이동평균 필터 저장 변수
int transmission_center = 90;

const float velocity_weight[] = {0.0, 0.25, 0.5, 0.75,
                                 1.0, 1.25, 1.5, 1.75, 2.0}; // 속도 가중치 배열
int velocity_weight_num = 4;                                 // 속도 가중치 배열 인덱스

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
 * Subscribers
 *******************************************************************************/
ros::Subscriber<std_msgs::Int8> led_sub("led", messageLEDControlCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel_converted", commandVelocityCallback);

/*******************************************************************************
 * Publishers
 *******************************************************************************/
geometry_msgs::Twist cmd_vel_echo;
ros::Publisher cmd_vel_echo_pub("cmd_vel_echo", &cmd_vel_echo);
std_msgs::Float32 velocity_weight_echo;
ros::Publisher velocity_weight_echo_pub("velocity_weight_echo", &velocity_weight_echo);

/*******************************************************************************
 * SoftwareTimer of ZZANGDOL
 *******************************************************************************/
static uint32_t tTime[5];
static uint32_t t = 0; // 전역 시간 변수

/*******************************************************************************
 * Declaration for controllers
 *******************************************************************************/
const float zero_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float goal_velocity_from_rc[WHEEL_NUM] = {0.0, 0.0};

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
    nh.advertise(velocity_weight_echo_pub);

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

    pinMode(YELLOW_SWITCH, INPUT_PULLUP);
    pinMode(BLUE_SWITCH, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(YELLOW_SWITCH), ISR_WeightIncrease, FALLING);
    attachInterrupt(digitalPinToInterrupt(BLUE_SWITCH), ISR_WeightDecrease, FALLING);
    // pinMode(WEIGHT_COUNT, OUTPUT);

#if SERIAL_DEBUG
    Serial.begin(115200);
#endif
}

void loop()
{
    t = millis();
    // updateTime(); // ros time update

    /* motor contorol section : tTime[0] - MOTOR SIGNAL SEQUNCE */
    if ((t - tTime[0]) > (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
    {
        updateGoalVelocity(); // 특정 주기마다 goal velocity 업데이트

        if (!mode_rc && ((t - tTime[3]) > CONTROL_MOTOR_TIMEOUT)) // 일정시간 동안 입력이 없을 시, 속도 값을 0으로 설정.
        {
            goal_velocity_from_cmd[LINEAR] = zero_velocity[LINEAR];
            goal_velocity_from_cmd[ANGULAR] = zero_velocity[ANGULAR];
            goal_velocity[LINEAR] = zero_velocity[LINEAR];
            goal_velocity[ANGULAR] = zero_velocity[ANGULAR];
        }

        /* motor control data update */
        Driving_Servo.write((int)round(goal_velocity[LINEAR]) + 90);
        Steering_Servo.write((int)round(goal_velocity[ANGULAR]) + 90);
        tTime[0] = t;
    }

    /* publish cmd_vel_echo section - 특정 주기마다 echo cmd_vel 데이터 퍼블리시 : tTime[1] - CMD VEL ECHO MSG SIGNAL SEQUENCE */
    if ((t - tTime[1]) > (1000 / CMD_VEL_MSG_PUBLISH_FREQUENCY))
    {
        publishCmdVelEchoMsg();
        tTime[1] = t;
    }

    /* velocity weight echo section - velocity_weight 변수 값 1초마다 전송 : tTime[2] - VELOCITY WEIGHT MSG SIGNAL SEQUENCE*/
    if ((t - tTime[2]) > 1000)
    {
#if SERIAL_DEBUG
        Serial.print("velocity_weight :");
        Serial.print(velocity_weight);
        Serial.print("\n");
#endif
        velocity_weight_echo.data = velocity_weight[velocity_weight_num];
        velocity_weight_echo_pub.publish(&velocity_weight_echo);
        tTime[2] = t;
    }

    /* RC mode check section : tTime[3] - MOTOR MODE CHECK SEQUENCE */
    getDataFromRemoteController(); // RC컨트롤러로부터 제어 정보 지속적으로 업데이트
    if (mode_rc)
    {
        tTime[3] = t;
    }

    nh.spinOnce(); // Call all the callbacks waiting to be called at that point in time

    waitForSerialLink(nh.connected()); // Wait the serial link time to process
}

/*******************************************************************************
 * Publisher functions
 *******************************************************************************/
void publishCmdVelEchoMsg()
{
    cmd_vel_echo.linear.x = round(goal_velocity[LINEAR]);
    cmd_vel_echo.angular.z = round(goal_velocity[ANGULAR]);
    cmd_vel_echo_pub.publish(&cmd_vel_echo);
}

/*******************************************************************************
 * Callback functions
 *******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    /* 최대 최소 범위 안으로 입력받게끔 설정 */
    goal_velocity_from_cmd[LINEAR] = constrain(cmd_vel.linear.x, MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);     // 직선 주행 데이터 받기 및 cmd velocity 업데이트
    goal_velocity_from_cmd[ANGULAR] = constrain(cmd_vel.angular.z, MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY); // 각속도 주행 데이터 받기 및 cmd velocity 업데이트
    tTime[3] = millis();
    return;
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
static inline void getDataFromRemoteController()
{
    /* Trasmission Control */
    /******************************************************************/
    transmission_pulse_value = pulseIn(TRANSMISSION_RC_IN_PIN, HIGH); // 무선 수신기로부터 PWM 파형 받음. 펄스의 길이를 us 단위로 반환
    transmission_sum += (int)((transmission_pulse_value - 751) / 7.67) - transmission;
    transmission = (int)((transmission_sum) / MOVING_AVG_LEN);

    /* center value filter */
    transmission += 50;
    if (transmission_center + 5 < transmission)
        transmission_center = transmission - 1;
    if (transmission_center - 5 > transmission)
        transmission_center = transmission + 1;

    /* transmission 신호가 120보다 작을 경우 RC 모드로, 클 경우 자율주행 모드로*/
    transmission < 120 ? mode_rc = true : mode_rc = false;
    transmission -= 50;

    /* transmssion motor control */
    Transmission_Servo.write(135);

    /* Steering Control */
    /******************************************************************/
    steering_sum -= steering;
    steering_pulse_value = pulseIn(STEERING_RC_IN_PIN, HIGH); // 무선 수신기로부터 PWM 파형 받음. 펄스의 길이를 us 단위로 반환
    steering = (int)((steering_pulse_value - 751) / 7.67);

    /* check steering value validity */
    if (steering >= (90 + MAX_ANGULAR_VELOCITY))
        steering = (90 + MAX_ANGULAR_VELOCITY);
    else if (steering <= (90 + MIN_ANGULAR_VELOCITY))
        steering = 90 + MIN_ANGULAR_VELOCITY;
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

    /* Driving Control */
    /******************************************************************/
    driving_pulse_value = pulseIn(DRIVING_RC_IN_PIN, HIGH);
    driving = (int)((driving_pulse_value - 790) / 7.67);
    if (driving >= 90)
        driving = (int)((driving - 90) * 0.2 + 96); // 84보다 작으면 84로, 84보다 크면 96으로
    if (driving <= 96 && driving >= 80)
        driving = 90;

    /* check steering value validity */
    if (driving >= MAX_RC_VELOCITY)
        driving = MAX_RC_VELOCITY;
    else if (driving <= MIN_RC_VELOCITY)
        driving = MIN_RC_VELOCITY;

#if SERIAL_DEBUG
    Serial.print("raw data, driving : ");
    Serial.println(driving);
#endif

    /* FINAL VALUE UPDATE : rc컨트롤러로 부터 제어신호 값 업데이트 -> 정규화를 위해 -90 */
    /******************************************************************/
    goal_velocity_from_rc[LINEAR] = (float)driving - 90.0;
    goal_velocity_from_rc[ANGULAR] = (float)steering_center - 90.0;
    return;
}

/*******************************************************************************
 * Wait for Serial Link
 *******************************************************************************/
static inline void waitForSerialLink(bool isConnected)
{
    static bool wait_flag = false;
    if (isConnected && (wait_flag == false))
    {
        delay(10);
        wait_flag = true;
    }
    else
    {
        wait_flag = false;
    }
    return;
}

// /*******************************************************************************
//  * Update the base time for interpolation
//  *******************************************************************************/
// static inline void updateTime()
// {
//     current_offset = millis();
//     current_time = nh.now();
//     return;
// }

/*******************************************************************************
 * Update Goal Velocity
 *******************************************************************************/
static inline void updateGoalVelocity(void)
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

    /* weight 파라미터 조절 : 후진 시 goal_velocity_value < -10.0, 전진 시 goal_velocity_value > +6.0 */
    if (goal_velocity[LINEAR] > 0.0)
    {
        goal_velocity[LINEAR] = constrain(goal_velocity[LINEAR] - (5.0 * (1.0 - velocity_weight[velocity_weight_num])),
                                          7.0,
                                          MAX_LINEAR_VELOCITY);
    }
    return;
}

/*******************************************************************************
 * Huins-AI-Car Bundled Function
 *******************************************************************************/
void Buzzer_ON()
{
    digitalWrite(BUZZER_PIN, HIGH); // turn the LED on (HIGH is the voltage level)
    return;
}
void Buzzer_OFF()
{
    digitalWrite(BUZZER_PIN, LOW); // turn the LED off by making the voltage LOW
    return;
}
void Buzzer_ON_OFF()
{
    Buzzer_ON();
    delay(2); // wait for a second
    Buzzer_OFF();
    return;
}

/*******************************************************************************
 * Huins-AI-Car Bundled Function
 *******************************************************************************/
void ISR_WeightIncrease()
{
    if (velocity_weight_num < 8)
        velocity_weight_num += 1;
    /* weight LED preview with PWM : test codes
    velocity_weight_count++;
    delay(10);
    for (int i = 0; i < velocity_weight_count; i++)
    {
        digitalWrite(WEIGHT_COUNT, HIGH);
        delay(50000);
        digitalWrite(WEIGHT_COUNT, LOW);
        delay(50000);
    }
    */
    return;
}
void ISR_WeightDecrease()
{
    if (velocity_weight_num > 0)
        velocity_weight_num -= 1;
    /* weight LED preview with PWM : test codes
    velocity_weight_count--;
    delay(10);
    for (int i = 0; i < velocity_weight_count; i++)
    {
        digitalWrite(WEIGHT_COUNT, HIGH);
        delay(50000);
        digitalWrite(WEIGHT_COUNT, LOW);
        delay(50000);
    }
    */
    return;
}