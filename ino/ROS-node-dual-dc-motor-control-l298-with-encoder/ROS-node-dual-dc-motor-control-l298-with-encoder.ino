/*
 * Title: DC Motor Control with Encoder (Single) 
 * Author: Khairul Izwan 10-03-2020
 * Description: Controlling DC Motor with Encoder using L298P Motor Driver
 */

/* 
 *  Parts:  
 *  1. DC Motor:: 12V 38RPM 5kgfcm Brushed DC Geared Motor with Encoder :: https://my.cytron.io/p-12v-38rpm-5kgfcm-brushed-dc-geared-motor-with-encoder?search=12v%2038RPM&description=1&src=search.list
 *  2. Driver:: Shield L298P Motor Driver with GPIO :: https://my.cytron.io/p-shield-l298p-motor-driver-with-gpio?src=search.list
 */

/* 
 *  Notes:
 *  D10, D11, D12 and D13 for motor control
 *  D10 controls speed of motor A and D11 controls speed of motor B.
 *  Pin D12 controls direction of motor A and Pin D13 controls direction of motor B.
 */

//include necessary library
#include <ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//Pin Assignment
int PWMA = 10; // Left
int DIRA = 12;
int PWMB = 11; // Right
int DIRB = 13;

//Speed
int SPD1 = 50;

//Encoder Pins Definition
//Using an interrupt pins :: 2, 3, 18, 19, 20, 21 :: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/?setlang=it
#define ENCODER_COUNT_UP_A 2
#define ENCODER_COUNT_DOWN_A 3
#define ENCODER_COUNT_UP_B 18
#define ENCODER_COUNT_DOWN_B 19

//Encoder Variables
volatile signed long TEMP_A, COUNTER_A = 0;
volatile signed long TEMP_B, COUNTER_B = 0; 

//Change according to the robot wheel dimension
#define wheelSep 0.16 // in unit meter (m)
#define wheelRadius 0.023; // in unit meter (m)

//Variables declaration
float transVelocity;
float rotVelocity;

float leftVelocity;
float rightVelocity;

float leftDutyCycle;
float rightDutyCycle;

float leftPWM;
float rightPWM;

//Callback function for geometry_msgs::Twist
void messageCb_cmd_vel(const geometry_msgs::Twist &msg)
{
//  Get the ros topic value
  transVelocity = msg.linear.x;
  rotVelocity = msg.angular.z;
	
//	Differential Drive Kinematics::http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
//  Differential Drive Kinematics::https://snapcraft.io/blog/your-first-robot-the-driver-4-5

//  Step 1: Calculate wheel speeds from Twist
  leftVelocity = transVelocity - ((rotVelocity * wheelSep) / 2);
  rightVelocity = transVelocity + ((rotVelocity * wheelSep) / 2);
  
//  Step 2: Convert wheel speeds into duty cycles
  leftDutyCycle = (255 * leftVelocity) / 0.22;
  rightDutyCycle = (255 * rightVelocity) / 0.22;

//  Ensure DutyCycle is between minimum and maximum
  leftPWM = clipPWM(abs(leftDutyCycle), 0, 255);
  rightPWM = clipPWM(abs(rightDutyCycle), 0, 255);

//  motor directection helper function
  motorDirection();
}

//  Helper function to ensure DutyCycle is between minimum and maximum
float clipPWM(float PWM, float minPWM, float maxPWM)
{
  if (PWM < minPWM)
  {
    return minPWM;
  }
  else if (PWM > maxPWM)
  {
    return maxPWM;
  }
  return PWM;
}

/*Motor Direction helper function*/
void motorDirection()
{
  if (leftDutyCycle > 0 and rightDutyCycle > 0)
  {
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, HIGH);
    analogWrite(PWMA, leftPWM);
    analogWrite(PWMB, rightPWM);
  }
  else if (leftDutyCycle < 0 and rightDutyCycle < 0)
  {
    digitalWrite(DIRA, HIGH);
    digitalWrite(DIRB, LOW);
    analogWrite(PWMA, leftPWM);
    analogWrite(PWMB, rightPWM);
  }
  else if (leftDutyCycle < 0 and rightDutyCycle > 0)
  {
    digitalWrite(DIRA, HIGH);
    digitalWrite(DIRB, HIGH);
    analogWrite(PWMA, leftPWM);
    analogWrite(PWMB, rightPWM);
  }
  else if (leftDutyCycle > 0 and rightDutyCycle < 0)
  {
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, LOW);
    analogWrite(PWMA, leftPWM);
    analogWrite(PWMB, rightPWM);
  }
  else if (leftDutyCycle == 0 and rightDutyCycle == 0)
  {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }
}

/*
* ENCODER COUNTER FUNCTION
*/
void COUNT_INTERRUPT_LEFT_CW() 
{
  if(digitalRead(ENCODER_COUNT_DOWN_A)==LOW) 
  {
    COUNTER_A++;
  }
  else
  {
    COUNTER_A--;
  }
}

void COUNT_INTERRUPT_LEFT_CCW()
{
  if(digitalRead(ENCODER_COUNT_UP_A)==LOW) 
  {
    COUNTER_A--;
  }
  else
  {
    COUNTER_A++;
  }
}

void COUNT_INTERRUPT_RIGHT_CW() 
{
  if(digitalRead(ENCODER_COUNT_DOWN_B)==LOW) 
  {
    COUNTER_B++;
  }
  else
  {
    COUNTER_B--;
  }
}

void COUNT_INTERRUPT_RIGHT_CCW() 
{
  if(digitalRead(ENCODER_COUNT_UP_B)==LOW) 
  {
    COUNTER_B--;
  }
  else
  {
    COUNTER_B++;
  }
}

void messageCb_reset_encLeft(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_A = 0;
  }
}

void messageCb_reset_encRight(const std_msgs::Bool &msg)
{
  if (msg.data == true)
  {
    COUNTER_B = 0;
  }
}

//Set up the ros node (publisher and subscriber)
std_msgs::Float32 encLeft;
std_msgs::Float32 encRight;
ros::Publisher pub_encLeft("val_encLeft", &encLeft);
ros::Publisher pub_encRight("val_encRight", &encRight);

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("/cmd_vel", messageCb_cmd_vel);
ros::Subscriber<std_msgs::Bool> sub_reset_encLeft("/reset_encLeft", messageCb_reset_encLeft);
ros::Subscriber<std_msgs::Bool> sub_reset_encRight("/reset_encRight", messageCb_reset_encRight);

ros::NodeHandle nh;

//put your setup code here, to run once:
void setup()
{
/*	PinMode Setting*/
	pinMode(ENCODER_COUNT_UP_B, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_B, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_UP_A, INPUT_PULLUP);
	pinMode(ENCODER_COUNT_DOWN_A, INPUT_PULLUP);

/*	Interrupt Setting*/
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_A), COUNT_INTERRUPT_LEFT_CW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_A), COUNT_INTERRUPT_LEFT_CCW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_UP_B), COUNT_INTERRUPT_RIGHT_CW, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_COUNT_DOWN_B), COUNT_INTERRUPT_RIGHT_CCW, RISING);

	pinMode(DIRA, OUTPUT);
	pinMode(DIRB, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(PWMB, OUTPUT);

	nh.initNode();

	nh.advertise(pub_encLeft);
	nh.advertise(pub_encRight);

	nh.subscribe(sub_cmd_vel);

	nh.subscribe(sub_reset_encLeft);
	nh.subscribe(sub_reset_encRight);
}

//put your main code here, to run repeatedly:
void loop()
{
	encLeft.data = COUNTER_A;
	encRight.data = COUNTER_B;

	pub_encLeft.publish(&encLeft);
	pub_encRight.publish(&encRight);

	nh.spinOnce();
}
