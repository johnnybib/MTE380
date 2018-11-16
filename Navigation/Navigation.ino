#include <AutoPID.h>

#include <Encoder.h>

#include <DualVNH5019MotorShield.h>

#define KP 15
#define KI 0.0531
#define KD 0.02655
#define OUTPUT_MIN 0
#define OUTPUT_MAX 400


DualVNH5019MotorShield md;
//M1 positive forward - left motor
//N2 negative forward - right motor


const unsigned int TRIG_PIN_RIGHT=51;
const unsigned int TRIG_PIN_CENTER=49;
const unsigned int TRIG_PIN_LEFT=47;
const unsigned int ECHO_PIN_RIGHT=50;
const unsigned int ECHO_PIN_CENTER=48;
const unsigned int ECHO_PIN_LEFT=46;

const float FILTER_WEIGHT = 0.2;  
const float ENC_CONVERSION = 32.0;

const float ENCODER_PER_REV = 1632.67;
const float TURN_RADIUS = 50.8;
const float WHEEL_RADIUS = 40.0;

double leftSpeed, leftSetPoint, leftOutputVal;
double leftTimePrev;
double leftSpeedPrev;
int leftEncPrev;

double rightSpeed, rightSetPoint, rightOutputVal;
double rightTimePrev;
double rightSpeedPrev;
int rightEncPrev;


AutoPID leftPID(&leftSpeed, &leftSetPoint, &leftOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID rightPID(&rightSpeed, &rightSetPoint, &rightOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

Encoder leftEnc(19, 18);
Encoder rightEnc(20, 21);

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup()
{
  pinMode(TRIG_PIN_RIGHT,OUTPUT);
  pinMode(TRIG_PIN_CENTER,OUTPUT);
  pinMode(TRIG_PIN_LEFT,OUTPUT);
  pinMode(ECHO_PIN_RIGHT,INPUT);
  pinMode(ECHO_PIN_CENTER,INPUT);
  pinMode(ECHO_PIN_LEFT,INPUT);
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
  delayMicroseconds(100);
  TCCR2B &= ~_BV (CS22);
  TCCR2B |= _BV (CS21);
  

  leftTimePrev = 0;
  leftEncPrev = 0;
  leftSpeedPrev = 0;

  rightTimePrev = 0;
  rightEncPrev = 0;
  rightSpeedPrev = 0;

  delay(100);
  leftPID.setTimeStep(1);
  rightPID.setTimeStep(1);
}

int distance(unsigned int trig,unsigned int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  const unsigned long duration= pulseIn(echo, HIGH);
  return duration/29/2;
}

void turnLeft(unsigned int angle)
{
  float angleRad = angle * PI/180;
  unsigned int countsToTurn = (unsigned int)(ENCODER_PER_REV*(angleRad * TURN_RADIUS / WHEEL_RADIUS)/(2*PI))*2;
  Serial.println(countsToTurn);
  leftEnc.write(0);
  rightEnc.write(0);
//  md.setM1Speed(-200);
  md.setM2Speed(-200);
  while(abs(leftEnc.read()) < countsToTurn && abs(rightEnc.read()) < countsToTurn)
  {
    delay(2);
  }
  md.setM1Speed(0);
  md.setM2Speed(0);
}

double getLeftEncSpeed()
{
  int curEncVal = abs(leftEnc.read());
  double curTime = millis();
  if(curTime-leftTimePrev > 2)
  {
    double calcSpeed = (curEncVal-leftEncPrev)/((curTime-leftTimePrev));
    leftEncPrev = curEncVal;
    leftTimePrev = curTime;
    double filteredSpeed = FILTER_WEIGHT*calcSpeed + (1-FILTER_WEIGHT)*leftSpeedPrev;
    leftSpeedPrev = filteredSpeed;
    return filteredSpeed;    
  }
  return leftSpeedPrev;
}


double getRightEncSpeed()
{
  int curEncVal = abs(rightEnc.read());
  double curTime = millis();
  if(curTime-rightTimePrev > 2)
  {
    double calcSpeed = (curEncVal-rightEncPrev)/((curTime-rightTimePrev));
    rightEncPrev = curEncVal;
    rightTimePrev = curTime;
    double filteredSpeed = FILTER_WEIGHT*calcSpeed + (1-FILTER_WEIGHT)*rightSpeedPrev;
    rightSpeedPrev = filteredSpeed;
    return filteredSpeed;    
  }
  return rightSpeedPrev;
}
void setLeftSpeed(double target, int dir)
{
  leftSpeed = getLeftEncSpeed() * ENC_CONVERSION;
  leftSetPoint = abs(target);
  leftPID.run();
  md.setM1Speed(leftOutputVal * dir);
}

void setRightSpeed(double target, int dir)
{
  rightSpeed = getRightEncSpeed() * ENC_CONVERSION;
  rightSetPoint = abs(target);
  rightPID.run();
  md.setM2Speed(rightOutputVal * dir);
}


void loop()
{

  
//  int distance_center=distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
//  int distance_right=distance(TRIG_PIN_RIG?HT,ECHO_PIN_RIGHT);
//  int distance_left=distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
  
  stopIfFault();
  setLeftSpeed(75, -1);
  setRightSpeed(75, 1);
  Serial.println(leftSpeed);



  /*for(i=100;i<400;i+=10){
    md.setM1Speed(-i);
    md.setM2Speed(i);
    delay(1000);
  }
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(10);
  for(i=100;i<400;i+=10){
    md.setM1Speed(i);
    md.setM2Speed(-i);
    delay(1000);
  }*/

/*  if(distance_center!=0){
    Serial.println(distance_center);
    if(distance_center>20&&distance_center<100){
      md.setM1Speed(-distance_center);
      md.setM2Speed(distance_center);
    }
    if(distance_center<=20){
      md.setM1Speed(0);
      md.setM2Speed(0);
      while(1){}
    }
  }
  if(distance_right!=0){
    Serial.println(distance_right);
  }
  if(distance_left!=0){
    Serial.println(distance_left);
  }
    */

//State 1: finding the ramp
  //State 2: crossing the ramp
  //State 3: finding the goal post
  //State 4: stop at goal
  //State 5: return to ramp
  //State 6: cross ramp again
  //State 7: return to home base
}
