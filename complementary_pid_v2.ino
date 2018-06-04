#include "I2Cdev.h"
#include "MPU6050.h"
#include <IRremote.h>
#include "Wire.h"

MPU6050 accelgyro;

const int ENA = A6;
const int ENB = A7;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 11;
const int IN4 = 10;

unsigned long currentTime;
unsigned long timeNow;
unsigned long lastTime;

int output;
double errSum, lastErr;
float cycleTime = 0.004;  //in seconds
double kp = 1.4;      //1.405
double ki = 9.1;
double kd = 5.86;       
double accelData;
double error;
double dErr;
double accelTotal;
double gyroData;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int sumgx, sumgy, sumgz;
int sumax, sumay, sumaz;
int ayTotal;
float delayTimeCorrect;

static int setpoint = 0;
static int input = 0;
static int lastInput = 0;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  accelgyro.initialize();
  
  for (int i = 1; i < 65; i = i + 1) {
    accelgyro.getRotation(&gx, &gy, &gz);
    sumgx = sumgx + gx;                         // gx offset
  }
  sumgx = sumgx / 64;
  for (int i = 1; i < 65; i = i + 1) {
    accelgyro.getAcceleration(&ax, &ay, &az);
    sumay = sumay + ay;                         // ay offset
  }
  sumay = sumay / 64;
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT); 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);   
}

void loop()
{
  currentTime = micros();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelData = (ay-sumay)>>2;  // divided by 4
  accelTotal = accelData + setpoint;
  gyroData = (gx-sumgx)>>1;   // divided by 2
  input = (0.98*((lastInput)+((gyroData)*cycleTime)))+(0.02*(accelTotal));
  /*
   * the above takes the last reading of gyro (lastInput), and then takes the current 
   * reading * cycle time (to work out a turning movement in a time frame)
   * and then adds the two together. this works out how much it has turned since last time
   * this is then multiplied by 98% and then added to 2% of the current accel reading.
   */
  calcOutput();
  MotorL298();
  lastErr = error;
  lastInput = input;
  correctTime();
}

void correctTime()
{
  timeNow = micros();
  delayTimeCorrect = 4000-(timeNow - currentTime);
  delayMicroseconds(delayTimeCorrect);
}  

void calcOutput()
{
  error = setpoint - input;
  error = error;
  errSum = errSum + (error * cycleTime); //intergral part sum of errors past
  
  double dInput = input - lastInput;
  output = (kp * error) + (ki * errSum) - (kd * dInput);
}
  
void MotorL298()
{
  output/=2;
  if (output > 255) output = 255;    
  if (output < -255)output = -255;
 
  if (output < 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, (output*-1));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, (output*-1));
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, output);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, output);
    
  }
}
