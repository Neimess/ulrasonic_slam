#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

ros::NodeHandle nh;

const int trigPin1 = 3;
const int echoPin1 = 5;
const int trigPin2 = 7;
const int echoPin2 = 8;
const int trigPin3 = 13;
const int echoPin3 = 11;
const int servoPin = 9;

std_msgs::Float64MultiArray distanceArray;
std_msgs::Float64 DistanceCenter;
std_msgs::Float64 DistanceLeft;
std_msgs::Float64 DistanceRight;

ros::Publisher pub_sonar_center("sonar_center", &DistanceCenter);
ros::Publisher pub_sonar_left("sonar_left", &DistanceLeft);
ros::Publisher pub_sonar_right("sonar_right", &DistanceRight);
// ros::Publisher pub_sonar("sonar", &distanceArray);
Servo servo;

void servo_cb(const std_msgs::UInt16& cmd_msg) {
  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

void setup() {
  Serial.begin(57600);
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
  // nh.advertise(pub_sonar);
  nh.subscribe(sub);

  servo.attach(servoPin);
}

void loop() {
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

  // float array[] = {distanceLeft, distanceCenter, distanceRight};
  // distanceArray.data_length = sizeof(array) / sizeof(array[0]);
  // distanceArray.data = array;
  // pub_sonar.publish(&distanceArray);
  DistanceLeft.data = distanceLeft;
  DistanceCenter.data = distanceCenter;
  DistanceRight.data = distanceRight;
  pub_sonar_center.publish(&DistanceCenter);
  pub_sonar_left.publish(&DistanceLeft);
  pub_sonar_right.publish(&DistanceRight);
  nh.spinOnce();
  delay(100);
}
