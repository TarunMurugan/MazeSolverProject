#ifndef MazeBot_h
#define MazeBot_h

#include "Arduino.h"
#include "HCSR04.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


class MazeBot
{
  public:
    MazeBot(int left_mot[2],int right_mot[2],int ultrasonic_echo[4],int ultrasonic_trig);
    void begin();   
    void forward();
    void turn_right();
    void turn_left();
    bool wall_left();
    bool wall_right();
    bool wall_front();
    #define front 0
    #define right 1
    #define behind 2
    #define left 3
    int speed=200,proportionality_const=20,sidewall_dist=5;
    float imu_weight=0.5;


  private:
    int *_left, *_right, _ultrasonic_trig, *_ultrasonic_echo,i,j;
    void get_yaw();
    uint8_t devStatus;
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
      // return status after each device operation (0 = success, !0 = error)

    int flag=1;
    double bot_angle,initial_angle,time_check=0,rect_bot_angle;
// orientation/motion vars
    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
               // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float initial_yaw,current_yaw; 


// packet structure for InvenSense teapot demo
    uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
    
};

#endif