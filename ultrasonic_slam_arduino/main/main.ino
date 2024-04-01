#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3.h>
#include <Servo.h>
#include <PID_v1.h>
#include <ros/time.h>
#include <math.h>
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
#define SCANLOOP 40
#define PIN_ENCODER_RIGHT 2
#define PIN_ENCODER_LEFT 3

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter
unsigned long lastMilli = 0;

float wheelRad = 0.0325; // Whell radius, in m
float wheelBase = 0.100; // Whellbase, in m

int previousScanTime = 0; // Переменная для отслеживания времени последнего сканирования
byte scanInterval = 200;   // Интервал сканирования (в миллисекундах)
// SPEED, PID

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor 

float speed_ang=0, speed_lin=0;


const int numReadings = 3; // количество измерений для медианного фильтра
int readings[numReadings][3]; // массив для хранения последних измерений
int index = 0; // индекс текущего измерения
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
const float max_speed = 0.4;
const byte encoder_cpr = 120;                //Encoder ticks or counts per rotation

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position
const int last_scan_time = 0;
PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


// ---  ROS laser --- //
std_msgs::UInt16 DistanceCenter;
std_msgs::UInt16 DistanceLeft;
std_msgs::UInt16  DistanceRight;
geometry_msgs::Vector3 speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

// ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Publisher pub_sonar_center("sonar_center", &DistanceCenter);
ros::Publisher pub_sonar_left("sonar_left", &DistanceLeft);
ros::Publisher pub_sonar_right("sonar_right", &DistanceRight);

Servo servo;
void servo_cb(const std_msgs::UInt16& cmd_msg) {
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}


ros::Subscriber<std_msgs::UInt16> sub_servo("servo", servo_cb);


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
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  
  nh.advertise(pub_sonar_center);
  nh.advertise(pub_sonar_left);
  nh.advertise(pub_sonar_right);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  nh.subscribe(sub_servo);
  nh.subscribe(sub_wheel);
  servo.attach(servoPin);

  // Setting PID
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

  pinMode(PIN_ENCODER_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), encoderLeftMotor, FALLING);

  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), encoderRightMotor, FALLING);
}



int getMedian(int values[][3], int size, int sensorIndex) {
  int sortedValues[numReadings];
  
  // копируем значения для указанного датчика во временный массив
  for (int i = 0; i < size; i++) {
    sortedValues[i] = values[i][sensorIndex];
  }

  // сортируем массив значений
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (sortedValues[i] > sortedValues[j]) {
        // обмен значениями
        int temp = sortedValues[i];
        sortedValues[i] = sortedValues[j];
        sortedValues[j] = temp;
      }
    }
  }

  // выбираем медианное значение
  return sortedValues[size / 2];
}

void loop() {
  nh.spinOnce();
  if((millis() - last_scan_time) >= SCANLOOP) {
  scan();
  }
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
  lastMilli = millis();
  

  // speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
  // PID_leftMotor.Compute();
  // PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); //
  // if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
  //     leftMotor->setSpeed(0);
  //     leftMotor->run(BRAKE);
  //   }
  //   else if (speed_req_left == 0){                        //Stopping
  //     leftMotor->setSpeed(0);
  //     leftMotor->run(BRAKE);
  //   }
  //   else if (PWM_leftMotor > 0){                          //Going forward
  //     leftMotor->setSpeed(abs(PWM_leftMotor));
  //     leftMotor->run(BACKWARD);
  //   }
  //   else {                                               //Going backward
  //     leftMotor->setSpeed(abs(PWM_leftMotor));
  //     leftMotor->run(FORWARD);
  //   }
  
  // speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);    
  // PID_rightMotor.Compute();                                                 
  // // compute PWM value for right motor. Check constant definition comments for more information.
  // PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255); // 
  // MotorR(PWM_rightMotor);    
  // MotorL(PWM_leftMotor);
  // publishSpeed(LOOPTIME);
  }
}



void scan() {
  

  int distanceLeft = getDistance(trigPin1, echoPin1);
  int distanceCenter = getDistance(trigPin2, echoPin2);
  int distanceRight = getDistance(trigPin3, echoPin3);

  readings[index][0] = distanceLeft; // сохранение нового измерения датчика 1
  readings[index][1] = distanceCenter; // сохранение нового измерения датчика 2
  readings[index][2] = distanceRight; // сохранение нового измерения датчика 3
  index = (index + 1) % numReadings; // переход к следующему индексу
  
  int medianDistance1 = getMedian(readings, numReadings, 0);
  int medianDistance2 = getMedian(readings, numReadings, 1);
  int medianDistance3 = getMedian(readings, numReadings, 2);

  DistanceLeft.data = medianDistance1;
  DistanceCenter.data = medianDistance2;
  DistanceRight.data = medianDistance3;
  pub_sonar_center.publish(&DistanceCenter);
  pub_sonar_left.publish(&DistanceLeft);
  pub_sonar_right.publish(&DistanceRight);
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH);
  uint8_t distance = duration * 0.034 / 2;
  return distance;
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
  analogWrite(EN_R, abs(PWM));
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


//Left motor encoder counter
void encoderLeftMotor() {
  static int prevStateLeft = LOW;
  int currentState = digitalRead(PIN_ENCODER_LEFT);

  if (prevStateLeft == LOW && currentState == HIGH) {
    // Если произошло восхождение, то увеличиваем счетчик
    pos_left++;
  } else if (prevStateLeft == HIGH && currentState == LOW) {
    // Если произошло нисхождение, то уменьшаем счетчик
    pos_left--;
  }

  prevStateLeft = currentState;
}

//Right motor encoder counter
void encoderRightMotor() {
  static int prevStateRight = LOW;
  int currentState = digitalRead(PIN_ENCODER_RIGHT);

  if (prevStateRight == LOW && currentState == HIGH) {
    // Если произошло восхождение, то увеличиваем счетчик
    pos_right++;
  } else if (prevStateRight == HIGH && currentState == LOW) {
    // Если произошло нисхождение, то уменьшаем счетчик
    pos_right--;
  }

  prevStateRight = currentState;
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