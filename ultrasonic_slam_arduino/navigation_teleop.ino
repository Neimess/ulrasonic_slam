#include "ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>


ros::NodeHandle nh;


// Motor A connections
#define EN_L 6
#define IN1_L  2
#define IN2_L 12
// Motor B connections
#define EN_R 5
#define IN1_R 4
#define IN2_R 8
#define PWM_MIN 120
#define PWM_RANGE 255

double w_r=0, w_l=0;
double wheel_rad = 0.0325, wheel_sep = 0.100;
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

float wheel1 = 0;
float wheel2 = 0;

int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

float speed_act_left; // actual left wheel speed in m/s
float speed_act_right; // actual right wheel speed in m/s

int loopTime = 10;

// ** ROS callback & subscriber **
std_msgs::Float32 result;
ros::Publisher debug("debugger", &result);

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void velCallback(  const geometry_msgs::Twist& vel)
{
     speed_ang = vel.angular.z;
     speed_lin = vel.linear.x;

     w_r = speed_lin + speed_ang * (wheel_sep/2);
     w_l = speed_lin - speed_ang * (wheel_sep/2);

     wheel1 = mapPwm(fabs(w_r), PWM_MIN, PWM_RANGE);
     wheel2 = mapPwm(fabs(w_l), PWM_MIN, PWM_RANGE);


    // digitalWrite(IN1_L, w_l < 0);
    // digitalWrite(IN2_L, w_l > 0);
    // analogWrite(EN_L, wheel1);
    // digitalWrite(IN1_R, w_r < 0);
    // digitalWrite(IN2_R, w_r > 0);
    // analogWrite(EN_R, wheel2);
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);     //create a subscriber for ROS cmd_vel topic

// ** Setup **
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

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

void setup() {  

  nh.initNode();              // init ROS
  nh.subscribe(sub);          // subscribe to cmd_vel
  nh.advertise(debug);
  Motors_init();
}


void loop() {
    MotorR();
    MotorL();
    
    
    nh.spinOnce();        // make sure we listen for ROS messages and activate the callback if there is one

     } // end of timed loop    


void MotorL(){
  digitalWrite(IN1_L, w_l < 0);
  digitalWrite(IN2_L, w_l > 0);
  analogWrite(EN_L, wheel1);

}

void MotorR(){
  digitalWrite(IN1_R, w_r < 0);
  digitalWrite(IN2_R, w_r > 0);
  analogWrite(EN_R, wheel2);
}


