#include "Arduino.h"
#include "MazeBot.h"
#include "HCSR04.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL
#define led1 25
#define led1 26

extern MPU6050 mpu;
extern HCSR04 ultrasonic;


MazeBot::MazeBot(int mot_left[2],int mot_right[2],int ultrasonic_echo[4],int ultrasonic_trig)
{// motor-[dir,pwm], ultrasonic-[front,right,behind,left]-[trig,echo]
    _left=mot_left;
    _right=mot_right;
    _ultrasonic_trig=ultrasonic_trig;
    _ultrasonic_echo=ultrasonic_echo;
}

void MazeBot::begin()
{  
  pinMode(_ultrasonic_trig,OUTPUT);
  pinMode(25,OUTPUT);
  pinMode(26,OUTPUT);
  for(i=0;i<2;i++)
  {
    pinMode(_left[i],OUTPUT);
    analogWrite(_left[i],0);
    pinMode(_right[i],OUTPUT);
    analogWrite(_right[i],0);
    
    
  }
  for(i=0;i<4;i++)
  {
    pinMode(_ultrasonic_echo[i],INPUT);
  }

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);
    //;  // wait for Leonardo enumeration, others continue immediately
 // FIFO storage buffer

  mpu.initialize();
  
  devStatus = mpu.dmpInitialize();


  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    
    mpu.setDMPEnabled(true);
    
    mpuIntStatus = mpu.getIntStatus();
    
    packetSize = mpu.dmpGetFIFOPacketSize();
   
  }

  time_check = millis();
  
  while(flag==1)
  {
    get_yaw();
    Serial.println("callibrating");
  }

  


  
}

void MazeBot::forward()
{
  float start_dist_front=ultrasonic.dist(front),start_dist_back=ultrasonic.dist(behind),left_dist,right_dist;
  float offset;
  i=0;
  while(((start_dist_front-ultrasonic.dist(front)))/2<25)//+(ultrasonic.dist(behind)-start_dist_back)
  { left_dist=ultrasonic.dist(left);right_dist=ultrasonic.dist(right);
    offset=0;
    if(this->wall_left() && this->wall_right())
    {
      offset=(left_dist-right_dist);//add imu offset after testing    
    }
    else if(this->wall_left())
    {
      offset=left_dist-5;
    }
    else if(this->wall_right())
    {
      offset=5-right_dist;
    }
    analogWrite(_left[1],0);analogWrite(_right[1],0);
    analogWrite(_left[0],speed-offset*proportionality_const);analogWrite(_right[0],speed+offset*proportionality_const);
    //Serial.print(offset);Serial.print(",");Serial.print(speed-offset*proportionality_const);Serial.print(",");Serial.println(speed+offset*proportionality_const);
    Serial.print(left_dist);Serial.print(",");Serial.print(right_dist);Serial.print(",");Serial.println(ultrasonic.dist(front));
  }
  analogWrite(_left[0],0);analogWrite(_right[0],0);
  
}

void MazeBot::get_yaw()
{
  // mpuInterrupt = false;
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;
    float ypr[3];
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    mpu.resetFIFO();
  

  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
    if ((millis() - time_check) > 15000)
    {
      if (flag == 1) 
      {
        initial_angle = ypr[0] * 180 / M_PI;
        flag = 0;
      }
    bot_angle = ypr[0] * 180 / M_PI - initial_angle;
    double rec_angle=int(bot_angle)%360;
    if(rec_angle>180)
    {
      rec_angle-=360;
    }

  
    if(rec_angle<-180)
    {
      rec_angle+=360;
    }   
      bot_angle=rec_angle;
  }
}

}

void MazeBot::turn_left()
{ 
  this->get_yaw();
  initial_yaw=bot_angle;
  do{ 
    this->get_yaw();
    rect_bot_angle=bot_angle;
    if(initial_yaw<0 && bot_angle>0)
    {
      rect_bot_angle=bot_angle-360;
    }
  
    //analogWrite(_left[1],120);analogWrite(_right[0],120);
    Serial.println(initial_yaw-rect_bot_angle);
    }while(initial_yaw-rect_bot_angle<90);
    //analogWrite(_left[1],0);analogWrite(_right[0],0);
}

void MazeBot::turn_right()
{ 
  this->get_yaw();
  initial_yaw=bot_angle;
  do{ 
    this->get_yaw();
    rect_bot_angle=bot_angle;
    if(initial_yaw>0 && bot_angle<0)
    {
      rect_bot_angle=bot_angle+360;
    }
    analogWrite(_left[0],120);analogWrite(_right[1],120);
    Serial.println(rect_bot_angle-initial_yaw);
    }while(rect_bot_angle-initial_yaw<90);
    analogWrite(_left[0],0);analogWrite(_right[1],0);
}

bool MazeBot::wall_left()
{
  if(ultrasonic.dist(left)<10)
  {
    return true;
  }
  return false;
}

bool MazeBot::wall_right()
{
  if(ultrasonic.dist(right)<10)
  {
    return true;
  }
  return false;
}

bool MazeBot::wall_front()
{
  if(ultrasonic.dist(front)<10)
  {
    return true;
  }
  return false;
}
