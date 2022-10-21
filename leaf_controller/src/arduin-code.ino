//Sabtertooth motor controller imports
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <ros.h>
#include <std_msgs/Float64.h>
//target in degrees, calculated from ros topic "steering_angle"
float target = 0.0;

const int encoderPinA = 2;
const int encoderPinB = 3;

//The number of pulses produced by the encoder within a revolution.
const int PPR = 1000;

//The value is '1' if the encoder is not attached to any motor
const int gearRatio = 24;
const int decodeNumber = 10;

//record the current number of pulses received
volatile long int currentPosition = 0;

//rotation angle in degrees
double rotationalAngle = 0.0;


long prevT = 0;
float eprev = 0;
float eintegral = 0;

//ros subscriber code
ros::NodeHandle nh;
void messageCb(const std_msgs::Float64& msg)
{
  target = msg.data * 180 / 3.141;
}
ros::Subscriber<std_msgs::Float64> sub("steering_angle", &messageCb);
SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

void setup() {
  // put your setup code here, to run once:
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);
  Serial.begin(38400);
  SWSerial.begin(9600);

  //ros init
  nh.initNode();
  nh.subscribe(sub);


}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  rotationalAngle = 0.086*(360 * currentPosition) / (PPR * decodeNumber * gearRatio);
  Serial.println(rotationalAngle);


  String rotationalAngle_log_string = "rotation reading " + String(rotationalAngle);
//  /const char *rotationalAngle_log_char = rotationalAngle_log_string.c_str();
//  nh.loginfo(rotationalAngle_log_char);
  String target_log_string = " | target " + String(target);
//  /const char *target_log_char = target_log_string.c_str();
  String logging = rotationalAngle_log_string + target_log_string;
  const char *logging_char = logging.c_str();
  nh.loginfo(logging_char);

  // PID constants, need to tune
  float kp = 10;
  float kd = 0.02;
  float ki = 0;
// float kp = 0.02;
//  float kd = 0.002;
//  float ki = 0.1;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // error
  float e = rotationalAngle - target;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if ( pwr > 80 ) {
    pwr = 80;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  if (u == 0) {
    dir = 0;
  }

  // signal the motor
  setMotor(dir, pwr);


  // store previous error
  eprev = e;

}

void doEncoderA()
{
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}

void doEncoderB()
{
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }

}
void setMotor(int dir, int pwr) {
  if (dir == 1) {

    ST.motor(1, pwr);  // Go forward cw

  }
  else if (dir == -1) {

    ST.motor(1, (-1*pwr));  // Go backwards acw

  }
  else {

    ST.motor(1, 0);  // Hold current position
  }
}
