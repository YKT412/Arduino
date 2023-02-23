#define MPU_INIT 1

#ifndef PS2_INIT

#endif

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
long tm = 0;

volatile bool mpuInterrupt = false;

unsigned long dt=0,curr=0,prev=0;
float previous_error = 0, e;
float P = 0;
float I = 0, D = 0, z = 0;

float yaw = 0;
float reference = 0;

float Kp = 155, Ki = 0, Kd = 370; //0.5//with planetary motors
float my_Ki = Ki;

int reset_ref = 1;
int first_time = 1;

int actual=0;
int desired=0;


void dmpDataReady()
{
  mpuInterrupt = true;
}

float return_yaw()
{
  while (!mpuInterrupt && fifoCount < packetSize);
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.resetFIFO();
    return (ypr[0] * 180 / M_PI);
  }

}

float ret_yaw()
{
  //  Serial.println(millis() - tm);
  //  tm = millis();
  float temper = return_yaw();

  if (reference >= 0 && reference <= 180)
  {
    return ((temper - reference) < -180 ? temper - reference + 360 : temper - reference);
  }
  else if (reference >= -180 && reference < 0)
  {
    return ((temper - reference) > 180 ? temper - reference - 360 : temper - reference);
  }
}

void reset_para()
{
  yaw = e = previous_error = P = I = D = z = reference = 0;
}

void set_mpu()
{
  int kam = 0;
  if (first_time == 1)
  {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    //    TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(112);
    mpu.setYGyroOffset(-78);
    mpu.setZGyroOffset(-16);
    mpu.setXAccelOffset(-6739);
    mpu.setYAccelOffset(-1779);
    mpu.setZAccelOffset(481);


//-6739  -1779 481 112 -78 -16




    if (devStatus == 0)
    {
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(digitalPinToInterrupt(19), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      while (1);
      Serial.println(F(")"));
    }
    kam = 0;
    while (kam < 100)
    {
      float temp = ret_yaw();
      //      ps2_data();
      kam++;
      Serial.println(kam);
    }
    reference = ret_yaw();
    first_time = 0;
  }
  else
  {
    kam = 0;
    reset_para();
    while (kam < 5)
    {
      float temp = ret_yaw();
      kam++;
    }
    reference = ret_yaw();
  }
}

void pid_mpu()
{
  actual = ret_yaw();
  e=(desired-actual);
  D = Kd*(e-previous_error);
  P=Kp*e;
  P+=D;
 previous_error= e;
  //Serial.println(P);
 
  }
  void reset()
{
  yaw = e = previous_error = P = I = D = z = reference = 0;
}
