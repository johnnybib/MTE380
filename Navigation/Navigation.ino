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
//M2 negative forward - right motor


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

unsigned int state=0;
unsigned int forwardSpeed=50;

double leftSpeed, leftSetPoint, leftOutputVal;
double leftTimePrev, leftSpeedPrev;
int leftEncPrev;
int leftDirection, leftAngle;
bool turningLeft;

double rightSpeed, rightSetPoint, rightOutputVal;
double rightTimePrev, rightSpeedPrev;
int rightEncPrev;
int rightDirection, rightAngle;
bool turningRight;


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
  leftDirection = 1;
  turningLeft = false;
  leftPID.setTimeStep(1);

  rightTimePrev = 0;
  rightEncPrev = 0;
  rightSpeedPrev = 0;
  rightDirection = -1;
  turningRight = false;
  rightPID.setTimeStep(1);

  delay(100);
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

//void findPost()
//{
//  //turnLeft(30); Turn to no longer be facing into the ramp
//  int[]Readings= new int[5];
//  int sum=0;
//  int average=0;
//  int tmp;
//  bool atPost=false;
//  for(int i=0;i<5;i++){
//    tmp=10000;
//    while(tmp>500){
//      tmp=distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
//    }
//    Readings[i]=tmp;
//    sum+=Readings[i];
//  }
//  average=sum/5;
//  
//  while(!atPost){
//    //turnLeft(5); Correct name to be whatever the turning function is
//    tmp=10000;
//    while(tmp>500){
//      tmp=distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
//    }
//    while(tmp<average-15&&!atPost){
//      atPost=runTowards(tmp);
//      while(tmp>500){
//        tmp=distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
//      }
//    }
//    else{
//      average+=(tmp-Readings[i])/5
//      Readings[i]=tmp;
//      i++;
//    }
//  }
//}

//Test loop
//void loop()
//{
//  stopIfFault();
//  setLeftSpeed(75, -1);
//  setRightSpeed(75, 1);
//  Serial.println(leftSpeed);
//}

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
  leftSetPoint = abs(target);
  leftDirection = dir;
}

void setRightSpeed(double target, int dir)
{
  rightSetPoint = abs(target);
  rightDirection = dir;
}

void updatePID()
{
  leftSpeed = getLeftEncSpeed() * ENC_CONVERSION;
  rightSpeed = getRightEncSpeed() * ENC_CONVERSION;
  leftPID.run();
  rightPID.run();
  md.setM1Speed(leftOutputVal * leftDirection);
  md.setM2Speed(rightOutputVal * rightDirection);
}

void turnLeft(unsigned int angle)
{
  float angleRad = angle * PI/180;
  unsigned int countsToTurn = (unsigned int)(ENCODER_PER_REV*(angleRad * TURN_RADIUS / WHEEL_RADIUS)/(2*PI))*2;
  int leftCurrentEnc = leftEnc.read();
  int rightCurrentEnc = rightEnc.read();
  setLeftSpeed(50, -1);
  setRightSpeed(50, -1);
  while(abs(leftEnc.read()) < countsToTurn+leftCurrentEnc && abs(rightEnc.read()) < countsToTurn+rightCurrentEnc)
  {
    updatePID();
  }
  md.setM1Speed(0);
  md.setM2Speed(0);
}


void loop()
{
  
}
