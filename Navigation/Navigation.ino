#include <AutoPID.h>

#include <Encoder.h>

#include <DualVNH5019MotorShield.h>
#include <Servo.h> 
#include <SoftwareSerial.h>


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
Servo myservo;

const unsigned int TRIG_PIN_RIGHT=51;
const unsigned int TRIG_PIN_CENTER=49;
const unsigned int TRIG_PIN_LEFT=47;
const unsigned int TRIG_PIN_BACK=43;
const unsigned int ECHO_PIN_RIGHT=50;
const unsigned int ECHO_PIN_CENTER=48;
const unsigned int ECHO_PIN_LEFT=46;
const unsigned int ECHO_PIN_BACK=42;


const float FILTER_WEIGHT = 0.2;  
const float LEFT_ENC_CONVERSION = 28;
const float RIGHT_ENC_CONVERSION = 28.3;

const float ENCODER_PER_REV = 1632.67;
const float TURN_RADIUS = 50.8;
const float WHEEL_RADIUS = 40.0;
const double ULTRASONIC_SEPARATION = 168;

unsigned int state = 3;
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

double distToPost = 0;


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
  myservo.attach(39);

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
  
  myservo.write(90);  // set servo to mid-point
  delay(100);

#ifdef TEST
moveAlignDist(20);
#endif
}

double distance(unsigned int trig,unsigned int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  const unsigned long duration = pulseIn(echo, HIGH);
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
  float correctionFactor = 1.05;
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

//Distance to move in cm
void movePID(int distanceToMove)
{
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

void alignTurn(double leftDist, double rightDist, double threshold)
{
  double turnAngle;
  bool turnLeft = false;
  while(abs(leftDist-rightDist) > threshold)
  {
    turnAngle = atan2(abs(leftDist-rightDist)*10, ULTRASONIC_SEPARATION)*180/PI;
    Serial.print("Left: ");
    Serial.println(leftDist);
    Serial.print("Right: ");
    Serial.println(rightDist);
    Serial.print("Angle: ");
    Serial.println(turnAngle);
    if(leftDist < rightDist)
    {
      Serial.println("left");
      turnLeft = true;
    }
    else if(leftDist > rightDist)
    {
      Serial.println("right");
      turnLeft = false;
    }
    if(turnAngle > 45)
    {
      Serial.println("shit");
      turnAngle = 10; 
      turnLeft = !turnLeft;     
    }
    turn(turnAngle, turnLeft);
    delay(1000);
    leftDist = distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  }
  
}
void alignFront(int targetDistance, double threshold)
{
  double leftDist;
  double rightDist;
  bool checkAgain = true;
  while(checkAgain)
  {
    leftDist = distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
    rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    alignTurn(leftDist, rightDist, threshold);
    if(abs(leftDist - targetDistance) > threshold || abs(rightDist - targetDistance))
    {
      movePID((distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT) + distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT))/2.0 - targetDistance);        
      delay(500);
    }
    else
    {
      checkAgain = false;    
    }
  }

}

void moveForwardAlign(int targetDistance)
{
  double leftDist = 5000;
  double rightDist = 5000;
  double degPerEnc = 0.0864696911;
  double turnAngle;
  int encoderDiff;
  unsigned long timer = millis();  
  leftEnc.write(0);
  rightEnc.write(0);
  md.setSpeeds(forwardSpeed, -forwardSpeed);
  while(leftDist > targetDistance || rightDist > targetDistance){
    if(millis() - timer > 500)
    {
      stopMotors();
      delay(500);
      encoderDiff = abs(leftEnc.read())-abs(rightEnc.read());
      Serial.print("Diff: ");
      Serial.println(encoderDiff);
      turnAngle = abs(encoderDiff)*degPerEnc;
      Serial.print("Angle: ");
      Serial.println(turnAngle);
      if(encoderDiff > 0)
      {
        turn(turnAngle, true);//turn left
      }
      else
      {
        turn(turnAngle, false);//turn right
      }
      delay(500);
//      leftEnc.write(0);
//      rightEnc.write(0);      
      timer = millis();
      md.setSpeeds(forwardSpeed, -forwardSpeed);
    }    
    leftDist = distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
    rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    Serial.println(leftDist);
  }
  stopMotors();
}

void moveBackward(int targetDistance)
{
  int leftDist = 5000;
  int rightDist = 5000;
  md.setSpeeds(-forwardSpeed, forwardSpeed);
  while(leftDist < targetDistance || rightDist < targetDistance){
    leftDist = distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
    rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    Serial.println(rightDist);
  }
  stopMotors();
}
void moveForward(int targetDistance){
  int leftDist = 5000;
  int rightDist = 5000;
  md.setSpeeds(forwardSpeed, -forwardSpeed);
  while(leftDist > targetDistance || rightDist > targetDistance){
    leftDist = distance(TRIG_PIN_LEFT,ECHO_PIN_LEFT);
    rightDist = distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
    Serial.println(rightDist);
  }
  stopMotors();
}

void moveAlignDist(int distanceToMove)
{
  double degPerEnc = 0.0864696911;
  double turnAngle;
  int encoderDiff;
  int encoderTicksToMove = distanceToMove*10/WHEEL_RADIUS/(2*PI)*ENCODER_PER_REV;
  unsigned long timer = millis();  
  leftEnc.write(0);
  rightEnc.write(0);
  md.setSpeeds(forwardSpeed, -forwardSpeed);
  while(abs(leftEnc.read()) < encoderTicksToMove && abs(rightEnc.read()) < encoderTicksToMove)
  {
    if(millis() - timer > 500)
    {
      stopMotors();
      delay(500);
      encoderDiff = abs(leftEnc.read())-abs(rightEnc.read());
      Serial.print("Diff: ");
      Serial.println(encoderDiff);
      turnAngle = abs(encoderDiff)*degPerEnc;
      Serial.print("Angle: ");
      Serial.println(turnAngle);
      if(encoderDiff > 0)
      {
        turn(turnAngle, true);//turn left
      }
      else
      {
        turn(turnAngle, false);//turn right
      }
      delay(500);      
      timer = millis();
      md.setSpeeds(forwardSpeed, -forwardSpeed);
    }    
  }
  stopMotors();
}

void driveUpRamp()
{
  double backDist = 5000;
  unsigned long timer = millis();
  setLeftSpeed(rampSpeed, -1);
  setRightSpeed(rampSpeed, 1);
  while(millis() - timer < 15000)
  {
    delay(1);
    updatePID();
  }
//  unsigned long timerForStartMeasurement = millis();
//  setLeftSpeed(rampSpeed, -1);
//  setRightSpeed(rampSpeed, 1);
//  while(true)//backDist > 10)
//  {
//    backDist = distance(TRIG_PIN_BACK,ECHO_PIN_BACK);
//    Serial.println(backDist); 
////    if(millis() - timer > 500)
////    {
////      if(millis() - timerForStartMeasurement > 5000)
////      {
////        backDist = distance(TRIG_PIN_BACK, ECHO_PIN_BACK);    
////        if(backDist < 10)
////        {
////          delay(200);
////          backDist = distance(TRIG_PIN_BACK,ECHO_PIN_BACK);      
////        }   
////        Serial.println(backDist); 
////      }
////      timer = millis();
////    }
////    updatePID();
//  }
//  Serial.println("done");
  stopMotors();
}


double moveBackwardSense()
{
  //Get average
  //Move forward in small increments checking for lower values
  double leftDist = 5000;
  double rightDist = 5000;
  double degPerEnc = 0.0864696911;
  double turnAngle;
  int encoderDiff;
  double timer = millis();  

  double high[10];
  boolean foundPole = false;
  boolean foundHigh = false;
  
  int highCount = 0;
  int minHigh = 100;
  int minLow = 10;

  double avgHigh = 0;
  double avgLow = 0;
  double distThresh = 10;
  
  leftEnc.write(0);
  rightEnc.write(0);
  while(!foundHigh)
  {
    avgHigh += distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
    highCount++;  
    if(highCount >= minHigh)
    {
      avgHigh = avgHigh/highCount;
      Serial.print("avgHigh: ");
      Serial.println(avgHigh);
      foundHigh = true;
    }     
  }
  md.setSpeeds(-forwardSpeed, forwardSpeed);


 
  while(!foundPole){
    if(millis() - timer > 500)
    {
      stopMotors();
      delay(500);
      encoderDiff = abs(leftEnc.read())-abs(rightEnc.read());
      Serial.print("Diff: ");
      Serial.println(encoderDiff);
      turnAngle = abs(encoderDiff)*degPerEnc;
      Serial.print("Angle: ");
      Serial.println(turnAngle);
      if(encoderDiff > 0)
      {
        turn(turnAngle, false);//turn right
      }
      else
      {
        turn(turnAngle, true);//turn left
      }
      delay(500);  

      //check values
      boolean foundLow = false;
      int lowCount = 0;
      avgLow = 0;
      while(!foundLow)
      { 
        avgLow += distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
        lowCount++;  
        if(lowCount >= minLow)
        {
          avgLow = avgLow/lowCount;
          Serial.print("AvgLow: ");
          Serial.println(avgLow);
          foundLow = true;
          if(avgHigh - avgLow > distThresh)
          {
            Serial.println("Found pole");
            foundPole = true;
          }
        }     
      }

      timer = millis();
      md.setSpeeds(-forwardSpeed, forwardSpeed);
    }    
  }
  
  movePID(avgLow*0.130526);
  
  stopMotors();
  return avgLow;
}
//
//void moveForwardStop()
//{
//  double leftDist = 5000;
//  double rightDist = 5000;
//  double degPerEnc = 0.0864696911;
//  double turnAngle;
//  int encoderDiff;
//  double timer = millis();
//  boolean atPole = false;
//  int readingCount = 0;  
//  leftEnc.write(0);
//  rightEnc.write(0);
//  md.setSpeeds(forwardSpeed, -forwardSpeed);
//  while(!atPole){
//    if(millis() - timer > 1000)
//    {
//      stopMotors();
//      delay(500);
//      encoderDiff = abs(leftEnc.read())-abs(rightEnc.read());
//      Serial.print("Diff: ");
//      Serial.println(encoderDiff);
//      turnAngle = abs(encoderDiff)*degPerEnc;
//      Serial.print("Angle: ");
//      Serial.println(turnAngle);
//      if(encoderDiff > 0)
//      {
//        turn(turnAngle, true);//turn left
//      }
//      else
//      {
//        turn(turnAngle, false);//turn right
//      }
//      delay(500);
////      leftEnc.write(0);
////      rightEnc.write(0);      
//      timer = millis();
//      md.setSpeeds(forwardSpeed, -forwardSpeed);
//    }    
//    double tempDist = distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
//    if(tempDist < 1)
//    {
//      Serial.print("TempDist: ");
//      Serial.println(tempDist); 
//      tempDist = distance(TRIG_PIN_CENTER,ECHO_PIN_CENTER);
//    }
//    dist[readingCount] = tempDist;
//    Serial.println(dist[readingCount]); 
//    readingCount++;
//    if(readingCount == READINGS_ARRAY_SIZE)
//    {
//      readingCount--;
//      if((dist[readingCount-3]==1&&dist[readingCount-2]==1&&dist[readingCount-1]==1&&dist[readingCount]==1)||(dist[readingCount-3]>1000&&dist[readingCount-2]>1000&&dist[readingCount-1]>1000&&dist[readingCount]>1000)){
//        atPole=1;
//        Serial.println("At pole");  
//      }
//      readingCount = 0;
//    }
//  }
//  stopMotors();
//}


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
//  Serial.println(distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT));
  /* Run modes
   * 0: Move to xcm from wall
   * 1: Rotate while scanning
   * 2: Moving forwards while scanning
   * 3: Moving backwards while scanning
   * default: Finished running (stop)
   */
#else
  switch(state){
    case 0://Move towards back wall and align 25cm away then turn left
      moveForwardAlign(25);
      resetVals();
      delay(500);
      alignFront(25, 2);
      resetVals();
      delay(500);
      turn(90,true);
      resetVals();
      delay(500);
      state++;
      break;
    case 1://Move towards right wall and align 21cm away then turn right
      moveForwardAlign(21);
      resetVals();
      delay(500);
      alignFront(21, 1);
      resetVals();
      delay(500);
      turn(90,false);
      resetVals();
      delay(500);
      state++;
    case 2://Align with back wall 40cm away
      alignFront(40, 1);
      resetVals();
      delay(500);
      state++;
      break;      
    case 3://Drive up ramp. Move backwards 20cm. Turn left. Align with right wall 45 cm away
//      driveUpRamp();
      delay(500);
      movePID(20);
      delay(500);
      turn(90, true);
      resetVals();
      delay(500);
      alignFront(45, 2);
      resetVals();      
      state++;
      break;
    case 4://Move backwards until post detected. Turn right. Move forward until 10cm away
      delay(1000);
      myservo.write(0);
      delay(1000);
      distToPost = moveBackwardSense();
      resetVals();
      delay(500);
      turn(90, false);//turn right
      resetVals();
      myservo.write(80);
      delay(2000);
      moveAlignDist(distToPost);
      state++;
      break;
    case 5://Move backwards 20cm. Turn around 180. Align on wall at 25. Turn right
      movePID(-20);
      delay(500);
      turn(180, false);
      moveForwardAlign(25);
      resetVals();
      delay(500);
      alignFront(25, 2);
      resetVals();
      delay(500);
      turn(90,false);
      resetVals();
      delay(500);
      state++;
      break;
    case 6://Align on right wall 26 cm and turn left
      moveForwardAlign(25);
      resetVals();
      delay(500);
      alignFront(25, 1);
      resetVals();
      delay(500);
      turn(90,true);
      resetVals();
      delay(500);
      state++;
      break;
    case 7://Align on back wall 40cm
      alignFront(40, 1);
      resetVals();
      delay(500);
      state++;
      break;      
    case 8://Drive up ramp, move back 20 cm, turn right, align with right wall 45 cm
      driveUpRamp();
      delay(500);
      movePID(20);
      delay(500);
      turn(90, false);
      resetVals();
      delay(500);
      alignFront(45, 2);
      resetVals();      
      state++;
      break;
    case 9://Drive backwards until see post. Turn right. Drive until 10cm
      delay(1000);
      myservo.write(160);
      delay(1000);
      moveBackwardSense();
      delay(500);
      turn(90, true);//turn right
      resetVals();
      myservo.write(80);
      delay(2000);
      moveAlignDist(10);
      state++;
      break;  
    default:
      stopMotors();
      while(1){}
      break;
  }
#endif
}
