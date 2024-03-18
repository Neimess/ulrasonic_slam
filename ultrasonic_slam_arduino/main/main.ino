#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Servo.h>

ros::NodeHandle nh;


// Motor A connections
#define enA 6
#define in1 2
#define in2 12
// Motor B connections
#define enB 5
#define in3 4
#define in4 8

#define trigPin1 A0
#define echoPin1 A1
#define trigPin2 A2
#define echoPin2 A3
#define trigPin3 A4
#define echoPin3 A5
#define servoPin 9

std_msgs::Float32 DistanceCenter;
std_msgs::Float32 DistanceLeft;
std_msgs::Float32 DistanceRight;

geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

ros::Publisher pub_sonar_center("sonar_center", &DistanceCenter);
ros::Publisher pub_sonar_left("sonar_left", &DistanceLeft);
ros::Publisher pub_sonar_right("sonar_right", &DistanceRight);

Servo servo;

void servo_cb(const std_msgs::UInt16& cmd_msg) {
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

// ** ROS callback & subscriber **


void velCallback(  const geometry_msgs::Twist& vel)
{
     x = vel.linear.x;
     z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);     //create a subscriber for ROS cmd_vel topic

void setup() {
  Serial.begin(57600);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  nh.initNode();
  nh.advertise(pub_sonar_center);
  nh.advertise(pub_sonar_left);
  nh.advertise(pub_sonar_right);     
  nh.subscribe(sub);

  servo.attach(servoPin);
}

void move(int speed, float time, char* direction){
  time = time * 1000;
  int speed_left = speed;
  int speed_right = speed-10;
  analogWrite(enA, speed_right);
  analogWrite(enB, speed);
    switch (*direction) {
        case 'F':
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          break;
        case 'B':
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          break;
    } 
  delay(time);
}

void rotate(float angle) {
  analogWrite(enA, 255);
  analogWrite(enB, 245);
  float proportion = abs(angle) / 90.0;
  int delayTime = int(proportion * 165);
  if (angle <= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  delay(delayTime);
}
  
void stop(int time){
    time = time * 1000;
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(time);
}

void scan() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  long durationCenter = pulseIn(echoPin1, HIGH);
  float distanceCenter = durationCenter * 0.034 / 2;

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  long durationLeft = pulseIn(echoPin2, HIGH);
  float distanceLeft = durationLeft * 0.034 / 2;

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  long durationRight = pulseIn(echoPin3, HIGH);
  float distanceRight = durationRight * 0.034 / 2;

  DistanceLeft.data = distanceLeft;
  DistanceCenter.data = distanceCenter;
  DistanceRight.data = distanceRight;
  pub_sonar_center.publish(&DistanceCenter);
  pub_sonar_left.publish(&DistanceLeft);
  pub_sonar_right.publish(&DistanceRight);
}

void loop() {
  nh.spinOnce();
  move(150, 1, "F"); 
  stop(1);
  scan();
  
  
  
  delay(100);
}
