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
#define led2 26
#define switch1 36
#define switch2 39


extern MPU6050 mpu;
extern HCSR04 ultrasonic;




MazeBot::MazeBot(int mot_left[2],int mot_right[2],int ultrasonic_echo[4],int ultrasonic_trig[2])
{// motor-[dir,pwm], ultrasonic-[front,right,behind,left]-[trig,echo]
    _left=mot_left;
    _right=mot_right;
    _ultrasonic_trig=ultrasonic_trig;
    _ultrasonic_echo=ultrasonic_echo;
}

void MazeBot::begin()
{ for(int i=0;i<2;i++)
  {
  pinMode(_ultrasonic_trig[i],OUTPUT);
  }
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(switch1,INPUT);
  pinMode(switch2,INPUT);
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
 // wait for Leonardo enumeration, others continue immediately
 //FIFO storage buffer

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
  delay(500);

  time_check = millis();
  
  while(flag==1)
  {
    get_yaw();
    Serial.println("callibrating");
  }
  this->Led1(1);
  while(this->Switch2()!=true){}
  this->Led1(0);
  this->Led2(1);
  delay(1000);
  this->Led2(0);



  


  
}

void MazeBot::forward()
{ //this->get_yaw();
int i;
  float start_dist_front=ultrasonic.dist(front),start_dist_back,left_dist,right_dist,movedist,start_dist;
  delay(10);
  start_dist_back=ultrasonic.dist(behind);
  Serial.print("start dist front:");Serial.println(start_dist_front);
  initial_yaw=bot_angle;
  // for(int i=0;i<10;i++){
  //   start_dist_front+=ultrasonic.dist(right);
  // }
  // start_dist_front/=10;
  float offset,forward_error;
  int integral_prior = 0,integral,derivative,offset_prior;
  start_dist_back=start_dist_back<1?1000:start_dist_back;
  start_dist_front=start_dist_front<1?1000:start_dist_front;
  if(start_dist_back>start_dist_front && start_dist_front<180 && start_dist_front>0){
    Serial.println("if");
  i=0;
  movedist=start_dist_front;
  start_dist=start_dist_front;

  }
  else{
    Serial.println("else");
    i=1;
    start_dist=start_dist_back;
  }
  
  //+(ultrasonic.dist(behind)-start_dist_back)
  if(i==1 || i==0)
  {
  do{ delay(15);
    // this->get_yaw();
    // rect_bot_angle=bot_angle;
    // if(initial_yaw<0 && bot_angle>0)
    // {
    //   rect_bot_angle=bot_angle-360;
    // }
    // Serial.print("--------");Serial.println(rect_bot_angle);

    left_dist=ultrasonic.dist(left);right_dist=ultrasonic.dist(right);
    if(i==0){
      Serial.println("front");
      movedist=ultrasonic.dist(front);
      movedist=(movedist<2?start_dist:movedist);
      forward_error=25-(start_dist-movedist);
    }
    else{
      this->Led1(1);
      this->Led2(1);
      Serial.println("back");
      movedist=ultrasonic.dist(behind);
      movedist=(movedist<2?start_dist:movedist);
      forward_error=25+(start_dist-movedist);
    }
    
    offset=0;
    
    speed=60*forward_error/25+35;
    if(this->wall_left(left_dist))
    {
     offset=left_dist-5.5;
    }
    else if(this->wall_right(right_dist))
    {
      offset=5.5-right_dist;
      
    }
    offset=1*offset+0*rect_bot_angle;
    integral = integral_prior + offset * 0.025;
    derivative= (offset-offset_prior)/0.025;
    analogWrite(_left[1],0);analogWrite(_right[1],0);
    analogWrite(_left[1],speed-(offset*proportionality_const+integral_offset*integral+derivative_offset*derivative));analogWrite(_right[1],speed+(offset*proportionality_const+integral_offset*integral+derivative_offset*derivative)-10);
   // analogWrite(_left[1],50*(25-(start_dist_front-front_dist))/25+45);analogWrite(_right[1],50*(25-(start_dist_front-front_dist))/25+35);
    
    //Serial.print(offset);Serial.print(",");Serial.print(speed-(offset*proportionality_const+integral_offset*integral));Serial.print(",");Serial.println(speed+(offset*proportionality_const+integral_offset*integral));
    Serial.print(start_dist_front);Serial.print(",");Serial.println(movedist);//Serial.print(",");Serial.println((start_dist_front-(ultrasonic.dist(front))==0?start_dist_front:ultrasonic.dist(front)));
    //Serial.println(50*(25-(start_dist_front-front_dist))/25+20);
    offset_prior=offset;
    integral_prior = integral;
  }while((forward_error)>0 && movedist>6); //((start_dist_front-front_dist))<25 && front_dist>5
        this->Led1(0);
      this->Led2(0);
  }
  // else{
  //   unsigned long start_time=esp_timer_get_time();
  //   Serial.println("start");
  //    analogWrite(_left[1],0);analogWrite(_right[1],0);
  //   while(esp_timer_get_time()-start_time<1500000 && ultrasonic.dist(front)>5){
  //  analogWrite(_left[1],100);analogWrite(_right[1],90);
  //   }
   

  // }
  analogWrite(_left[1],0);analogWrite(_right[1],0);
  Serial.println("done");

    // analogWrite(_left[1],0);analogWrite(_right[1],0);
    // analogWrite(_left[1],100);analogWrite(_right[1],90);
    // delay(1500);
    // analogWrite(_left[1],0);analogWrite(_right[1],0);
    // delay(500);
    // analogWrite(_left[0],100);analogWrite(_right[0],100);
    // delay(1500);
    // analogWrite(_left[0],0);analogWrite(_right[0],0);
    // delay(500);

  
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
{ static int i=68;
  this->get_yaw();
  this->get_yaw();
  initial_yaw=bot_angle;
  do{ 
    this->get_yaw();
    rect_bot_angle=bot_angle;
    if(initial_yaw<0 && bot_angle>0)
    {
      rect_bot_angle=bot_angle-360;
    }
    // if(i!=1)
    // {
    analogWrite(_left[0],20*((90-(rect_bot_angle-initial_yaw))/90)+30);analogWrite(_right[1],20*((90-(rect_bot_angle-initial_yaw))/90)+20);
    Serial.println(initial_yaw-rect_bot_angle);
    // }
    // else
    // {
    //   i=2;
    // }
    }while(initial_yaw-rect_bot_angle<i);
    analogWrite(_left[0],0);analogWrite(_right[1],0);
    i=88;
}

void MazeBot::turn_right()
{
  static int i=78;
  this->get_yaw();
  initial_yaw=bot_angle;
  do{ 
    this->get_yaw();
    rect_bot_angle=bot_angle;
    if(initial_yaw>0 && bot_angle<0)
    {
      rect_bot_angle=bot_angle+360;
    }
    analogWrite(_left[1],20*((90-(rect_bot_angle-initial_yaw))/90)+30);analogWrite(_right[0],20*((90-(rect_bot_angle-initial_yaw))/90)+30);
    Serial.println(rect_bot_angle-initial_yaw);
    }while(rect_bot_angle-initial_yaw<i);
    analogWrite(_left[1],0);analogWrite(_right[0],0);
    i=88;

}

bool MazeBot::wall_left(double x)
{ float left_dist;
  if(x=1000)
  {
    left_dist=ultrasonic.dist(left);
  }
  else
  {
    left_dist=x;
  }
  //Serial.print("left:");Serial.println(left_dist);
  if(left_dist<10 &&  left_dist>0)
  {
    return true;
  }
  return false;
}

bool MazeBot::wall_right(double x)
{ float right_dist;
  if(x=1000)
  {
  right_dist=ultrasonic.dist(right);
  }
  else
  {
    right_dist=x;
  }
  //Serial.print("right:");Serial.println(right_dist);
  if(right_dist<10 && right_dist>0)
  {
    return true;
  }
  return false;
}

bool MazeBot::wall_front()
{ float front_dist=ultrasonic.dist(front);
  if(front_dist<10 && front_dist>0)
  {
    return true;
  }
  return false;
}

void MazeBot::printvals()
{
  // Serial.print("front:");Serial.println(ultrasonic.dist(front));
  // Serial.print("behind:");Serial.println(ultrasonic.dist(behind));
  // Serial.print("left:");Serial.println(ultrasonic.dist(left));
  // Serial.print("right:");Serial.println(ultrasonic.dist(right));
  int behind_dist =ultrasonic.dist(behind);
  if(behind_dist>10){
    this->Led1(1);
  }
  else{
    this->Led1(0);
  }
  if(behind_dist>20){
    this->Led2(1);
  }
  else{
    this->Led2(0);
  }
  // this->get_yaw();
  // Serial.println(bot_angle);

  //  analogWrite(_left[1],100);analogWrite(_right[1],90);
  //   delay(750);
  //   analogWrite(_left[1],0);analogWrite(_right[1],0);
    // delay(500);
    // analogWrite(_left[1],100);analogWrite(_right[1],100);
    // delay(250);
    // analogWrite(_left[1],0);analogWrite(_right[1],0);
    delay(20);
}

void MazeBot::Led1(bool x)
{
  digitalWrite(led1,x);
}
void MazeBot::Led2(bool y)
{
  digitalWrite(led2,y);
}
bool MazeBot::Switch1()
{
  if(digitalRead(switch1) ==0)
  {
    return true;
  }
  return false;
}
bool MazeBot::Switch2()
{
  if(digitalRead(switch2) ==0)
  {
    return true;
  }
  return false;
}
