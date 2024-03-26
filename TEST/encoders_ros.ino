#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <PID_v1.h>
#include <ros/time.h>

ros::NodeHandle nh;


// Motor A connections
#define EN_L 6
#define IN1_L  7
#define IN2_L 12
// Motor B connections
#define EN_R 5
#define IN1_R 4
#define IN2_R 8

#define trigPin1 A0
#define echoPin1 A1
#define trigPin2 A2
#define echoPin2 A3
#define trigPin3 A4
#define echoPin3 A5
#define servoPin 9

// --- Robot-specific constants ---
#define LOOPTIME                      100     //Looptime in millisecond

#define PIN_ENCODER_RIGHT 2
#define PIN_ENCODER_LEFT 3

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;

const float wheelDiam = 0.065; // Whell radius, in m
const float wheelBase = 0.100; // Whellbase, in m

int previousScanTime = 0; // Переменная для отслеживания времени последнего сканирования
byte scanInterval = 200;   // Интервал сканирования (в миллисекундах)
// SPEED, PID

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

float speed_ang=0, speed_lin=0;

// PID Parameters
const float PID_left_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const float PID_right_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 

const float speed_to_pwm_ratio = 0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const float min_speed_cmd = 0.282;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).
const float max_speed = 0.8;
const unsigned int pulsesperturn = 30;                //Encoder ticks or counts per rotation
unsigned int rpm_right = 0;
unsigned int rpm_left = 0;
volatile unsigned long pulses_1 = 0; // Количество импульсов для первого энкодера
volatile unsigned long pulses_2 = 0; // Количество импульсов для второго энкодера
unsigned long last_time_1 = 0; // Последнее время для первого энкодера
unsigned long last_time_2 = 0; // Последнее время для второго энкодера

static volatile unsigned long debounce_1 = 0; // Tiempo del rebote.
static volatile unsigned long debounce_2 = 0;
PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


// geometry_msgs::Vector3 speed_msg;                                //create a "speed_msg" ROS message
// ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

// ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

std_msgs::Float32 encoder1_msg;
ros::Publisher encoder1_pub("encoder1_data", &encoder1_msg);

std_msgs::Float32 encoder2_msg;
ros::Publisher encoder2_pub("encoder2_data", &encoder2_msg);

std_msgs::Float32 result;
ros::Publisher debugger("debug", &result);

// ** ROS callback & subscriber **

void velCallback(const geometry_msgs::Twist& vel)
{
     noCommLoops = 0;
     speed_ang = vel.angular.z;
     speed_lin = vel.linear.x;

     speed_req_right = speed_lin + speed_ang * (wheelBase/2);
     speed_req_left = speed_lin - speed_ang * (wheelBase/2);

}

ros::Subscriber<geometry_msgs::Twist> sub_wheel("cmd_vel" , velCallback);     //create a subscriber for ROS cmd_vel topic
// ** Setup **
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup() {
  nh.initNode();
  Motors_init();
  nh.advertise(encoder1_pub);
  nh.advertise(encoder2_pub);
  nh.advertise(debugger);
  // nh.advertise(debugger);
  // nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  nh.subscribe(sub_wheel);

  // Setting PID
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

  pinMode(PIN_ENCODER_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), encoderLeftMotor, CHANGE);

  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), encoderRightMotor, CHANGE);
}




void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME) { // вход в тайм-луп
    unsigned long current_time = millis();
    
    if (current_time - last_time_1 >= 1000) { // Обновляем данные каждую секунду
    float rpm_1 = (60.0 * 1000.0 * pulsesperturn) / (pulses_1 * (current_time - last_time_1));
    float velocity_1 = rpm_1 * 3.1416 * wheelDiam / 60.0; // Вычисляем скорость в [м/с]
    encoder1_msg.data = velocity_1;
    encoder1_pub.publish(&encoder1_msg);
    pulses_1 = 0;
    last_time_1 = current_time;
    }

    if (current_time - last_time_2 >= 1000) { // Обновляем данные каждую секунду
      float rpm_2 = (60.0 * 1000.0 * pulsesperturn) / (pulses_2 * (current_time - last_time_2));
      float velocity_2 = rpm_2 * 3.1416 * wheelDiam / 60.0; // Вычисляем скорость в [м/с]
      encoder2_msg.data = velocity_2;
      encoder2_pub.publish(&encoder2_msg);
      pulses_2 = 0;
      last_time_2 = current_time;
    }

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();
    PWM_leftMotor = constrain(((speed_req_left + sgn(speed_req_left) * min_speed_cmd) / speed_to_pwm_ratio) + (speed_cmd_left / speed_to_pwm_ratio), -255, 255);

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    PID_rightMotor.Compute();
    PWM_rightMotor = constrain(((speed_req_right + sgn(speed_req_right) * min_speed_cmd) / speed_to_pwm_ratio) + (speed_cmd_right / speed_to_pwm_ratio), -255, 255);
    
    MotorR(PWM_rightMotor);
    MotorL(PWM_leftMotor);
    result.data = PWM_leftMotor;
    debugger.publish(&result);
    lastMilli = millis();
  }
}




void Motors_init(){

 pinMode(EN_L, OUTPUT);

 pinMode(EN_R, OUTPUT);

 pinMode(IN1_L, OUTPUT);

 pinMode(IN2_L, OUTPUT);

 pinMode(IN1_R, OUTPUT);

 pinMode(IN2_R, OUTPUT);

 digitalWrite(EN_L, LOW);

 digitalWrite(EN_R, LOW);

 digitalWrite(IN1_L, LOW);

 digitalWrite(IN2_L, LOW);

 digitalWrite(IN1_R, LOW);

 digitalWrite(IN2_R, LOW);

}

void MotorL(int PWM){
  digitalWrite(IN1_L, speed_req_left < 0);
  digitalWrite(IN2_L, speed_req_left > 0);
  analogWrite(EN_L, abs(PWM));

}

void MotorR(int PWM){
  digitalWrite(IN1_R, speed_req_right < 0);
  digitalWrite(IN2_R, speed_req_right > 0);
  analogWrite(EN_R, abs(PWM - 10));
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//Left motor encoder counter
void encoderLeftMotor() {
  if(  digitalRead (PIN_ENCODER_LEFT) && (micros()-debounce_1 > 500)) { 
// Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        debounce_1 = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        pulses_1++;
        }  // Suma el pulso bueno que entra.
}

void encoderRightMotor(){
  if(  digitalRead (PIN_ENCODER_RIGHT) && (micros()-debounce_2 > 500)) { 
// Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        debounce_2 = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        pulses_2++;
        }  // Suma el pulso bueno que entra.
} 



// Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
// void publishSpeed(double time) {
//   speed_msg.x = speed_act_left;    //left wheel speed (in m/s)
//   speed_msg.y = speed_act_right;   //right wheel speed (in m/s)
//   speed_msg.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
//   speed_pub.publish(&speed_msg);
//   nh.spinOnce();
//   nh.loginfo("Publishing odometry");
// }