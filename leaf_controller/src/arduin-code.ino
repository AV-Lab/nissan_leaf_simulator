#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

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

//ros imports
#include <ros.h>
#include <std_msgs/Float32.h>


ros::NodeHandle nh;
void messageCb(const std_msgs::Float32& msg)
  target = msg.data*180/3.141 
}
ros::Subscriber<std_msgs::Float32> sub("steering_angle", &messageCb);


volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {

  // encoder code
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);
  Serial.begin(9600);
  //SWSerial.begin(9600);
///////


  
 // Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
  /////ros init
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  // set target position
  //int target = 1200;
  // int target = 250*sin(prevT/1e6);
  //ros init
  nh.spinOnce();
  delay(2000);
  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  //encorder code
  rotationalAngle = (360 * currentPosition) / (PPR * decodeNumber * gearRatio);
  Serial.println(rotationalAngle);

 
  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  float pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = rotationalAngle;
  }
  
  // error
  float e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 40 ){
    pwr = 40;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  if(dir == 1){

    ST.motor(1, 40);  // Go forward cw
 
  }
  else if(dir == -1){

    ST.motor(1, -40);  // Go backwards acw

  }
  else{

    ST.motor(1, 0);  // Hold current position
  }
}

//void readEncoder(){
//  int b = digitalRead(ENCB);
//  if(b > 0){
//    posi++;
//  }
//  else{
//    posi--;
//  }
//}

void doEncoderA()
{
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    posi++;
  }
  else
  {
    posi--;
  }
}

void doEncoderB()
{
    if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
    {
      posi++;
    }
    else
    {
      posi--;
    }

  }
