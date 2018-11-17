#include <AutoPID.h>

#include <Encoder.h>

#include <DualVNH5019MotorShield.h>

#define ENABLE_DEBUG
//#define TEST

//#define KP_LEFT 5
//#define KI_LEFT 0.0531*200
//#define KD_LEFT 0.02655*10


#define KP_LEFT 8.4
#define KI_LEFT 0.0402
#define KD_LEFT 0.01005


#define KP_RIGHT 13.8
#define KI_RIGHT 0.0265
#define KD_RIGHT 0.006625

#define OUTPUT_MIN 0
#define OUTPUT_MAX 200


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
const float LEFT_ENC_CONVERSION = 28;
const float RIGHT_ENC_CONVERSION = 28.3;

const float ENCODER_PER_REV = 1632.67;
const float TURN_RADIUS = 50.8;
const float WHEEL_RADIUS = 40.0;

unsigned int state = 0;
unsigned int rampSpeed = 70;
unsigned int pidSpeed = 50;
unsigned int forwardSpeed = 120;
unsigned int turnSpeed = 50;

double leftSpeed, leftSetPoint, leftOutputVal;
double leftTimePrev, leftSpeedPrev;
int leftEncPrev;
int leftDirection, leftAngle;

double rightSpeed, rightSetPoint, rightOutputVal;
double rightTimePrev, rightSpeedPrev;
int rightEncPrev;
int rightDirection, rightAngle;


AutoPID leftPID(&leftSpeed, &leftSetPoint, &leftOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP_LEFT, KI_LEFT, KD_LEFT);
AutoPID rightPID(&rightSpeed, &rightSetPoint, &rightOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP_RIGHT, KI_RIGHT, KD_RIGHT);

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
  leftPID.setTimeStep(1);

  rightTimePrev = 0;
  rightEncPrev = 0;
  rightSpeedPrev = 0;
  rightDirection = -1;
  rightPID.setTimeStep(1);

  delay(100);

#ifdef TEST
//  setLeftSpeed(forwardSpeed, 1);
//  setRightSpeed(forwardSpeed, -1);
//md.setM1Speed(nonPIDSpeed);
//md.setM2Speed(-nonPIDSpeed);
//  turn(90, false);
//  resetVals();
//  delay(1000);
//  
//  turn(90, true);
//  delay(100);
#endif
}

double distance(unsigned int trig,unsigned int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  const unsigned long duration= pulseIn(echo, HIGH);
  return duration/29/2;
}

double getLeftEncSpeed()
{
  int curEncVal = abs(leftEnc.read());
  double curTime = millis();
  if(curTime-leftTimePrev > 10)
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
  if(curTime-rightTimePrev > 10)
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

//void rampSpeed(double target, int dir)
//{
//  setLeftSpeed(target, dir);
//  setRightSpeed(target, dir*-1);
//  int steadyStateTime = 0;
//  do
//  {
//    updatePID();
//    if(leftPID.atSetPoint(10) && rightPID.atSetPoint(10))
//    {
//      steadyStateTime++;
//    }
//    else
//    {
//      steadyStateTime = 0;
//    }
//  }while(steadyStateTime < 100);
////  md.setSpeeds(target * dir, target * -dir);
//}

void stopMotors()
{
  setLeftSpeed(0, 1);
  setRightSpeed(0, 1);
  md.setBrakes(0, 0);
}

void updatePID()
{
  leftSpeed = getLeftEncSpeed() * LEFT_ENC_CONVERSION;
  rightSpeed = getRightEncSpeed() * RIGHT_ENC_CONVERSION;
//  Serial.print("Left: ");
//  Serial.println(leftSpeed);
//  Serial.print("Right: ");
//  Serial.println(rightSpeed);
  leftPID.run();
  rightPID.run();
  md.setSpeeds(leftOutputVal * leftDirection, rightOutputVal * rightDirection);
}

void turn(unsigned int angle, bool left)
{
  float angleRad = angle * PI/180;
  float correctionFactor = 1.0;
  int countsToTurn = (unsigned int)(ENCODER_PER_REV*(angleRad * TURN_RADIUS*2 / WHEEL_RADIUS)/(2*PI))*correctionFactor;
  int leftCurrentEnc = leftEnc.read();
  int rightCurrentEnc = rightEnc.read();
  if(left)
  {
      setRightSpeed(turnSpeed, -1);    
      while(rightEnc.read() < (countsToTurn+rightCurrentEnc))
      {
        updatePID();
      }    
  }
  else
  {
      setRightSpeed(turnSpeed, 1);
      while(rightEnc.read() > (rightCurrentEnc-countsToTurn))
      {
        updatePID();
      }    
  }

  stopMotors();
}

void movePID(int targetDistance){
  int currentDistance = distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
  int distanceToMove = currentDistance - targetDistance;
  int encoderTicksToMove = distanceToMove*10/WHEEL_RADIUS/(2*PI)*ENCODER_PER_REV;
  leftEnc.write(0);
  rightEnc.write(0);
  int dir = 1;
  if(encoderTicksToMove < 0)
  {
    dir = -1; 
  }
  setLeftSpeed(pidSpeed, 1 * dir);
  setRightSpeed(pidSpeed, -1 * dir);
  while(abs(leftEnc.read()) < encoderTicksToMove*dir && abs(rightEnc.read()) < encoderTicksToMove*dir)
  {
    updatePID();
  }
  stopMotors();
}

void alignFront(int targetDistance)
{
  double leftDist;
  double rightDist;
  double threshold = 2.0;
  double turnAngle;
  bool checkAgain = true;
  while(checkAgain)
  {
    leftDist = distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    while(abs(leftDist-rightDist) > threshold)
    {
      turnAngle = 0.869668 * abs(leftDist - rightDist)*10;
      Serial.print("Left: ");
      Serial.println(leftDist);
      Serial.print("Right: ");
      Serial.println(rightDist);
      Serial.print("Angle: ");
      Serial.println(turnAngle);
      if(turnAngle > 90)
      {
        Serial.println("shit");
        turnAngle = 90;      
      }
      if(leftDist < rightDist)
      {
        Serial.println("left");
        turn(turnAngle, true);//turn left 
      }
      else if(leftDist > rightDist)
      {
        Serial.println("right");
        turn(turnAngle, false);//turn right
      }
      delay(2000);
      leftDist = distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
      rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    }
    if(abs(leftDist - targetDistance) > threshold)
    {
      movePID(targetDistance);
      delay(2000);
    }
    else
    {
      checkAgain = false;    
    }
  }

}

void moveBackward(int targetDistance)
{
  int tmp=5000;
  md.setSpeeds(-forwardSpeed, forwardSpeed);
  while(tmp<targetDistance){
    tmp=distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
    Serial.println(tmp);
  }
  stopMotors();
}
void moveForward(int targetDistance){
  int tmp=5000;
  md.setSpeeds(forwardSpeed, -forwardSpeed);
  while(tmp>targetDistance){
    tmp=distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
    Serial.println(tmp);
  }
  stopMotors();
}

void debug(String msg) {
  #ifdef ENABLE_DEBUG
    Serial.println(msg);
  #endif
}

void resetVals()
{
  leftTimePrev = 0;
  leftEncPrev = 0;
  leftSpeedPrev = 0;
  leftEnc.write(0);
  leftPID.reset();
 
  rightTimePrev = 0;
  rightEncPrev = 0;
  rightSpeedPrev = 0;
  rightEnc.write(0);
  rightPID.reset();
  
}

void loop()
{
#ifdef TEST
//  updatePID();
//  delay(1);
//leftSpeed = getLeftEncSpeed() * LEFT_ENC_CONVERSION;
//Serial.println(leftSpeed);
// rightSpeed = getRightEncSpeed() * RIGHT_ENC_CONVERSION;
// Serial.println(rightSpeed);
  Serial.println(distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT));
  /* Run modes
   * 0: Move to xcm from wall
   * 1: Rotate while scanning
   * 2: Moving forwards while scanning
   * 3: Moving backwards while scanning
   * default: Finished running (stop)
   */
#else
  switch(state){
    case 0:
      moveForward(35);
      resetVals();
      delay(500);
      debug("Moved Forward 35");
      alignFront(35);
      resetVals();
      delay(500);
      turn(90,true);
      resetVals();
      delay(500);
      debug("Turned left 90");
      state++;
      break;
//    case 1:
//      moveForward(8);
//      resetVals();
//      delay(500);
//      debug("Moved Forward 8");
//      turn(90,false);
//      resetVals();
//      delay(500);
//      debug("Turned right 90");
//      state++;
//    case 2:
//      setLeftSpeed(forwardSpeed, -1);
//      setRightSpeed(forwardSpeed, 1);
//      while(true)
//      {
//        updatePID();
//      }
    default:
      stopMotors();
      while(1){}
      break;
  }
#endif
}
