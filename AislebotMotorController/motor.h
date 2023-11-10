/*
Note: All the motor variables are in motor frame

*/
#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h" 


#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
//#define sgn(x) ((x) < 0 ? -1 : 1)

extern const float gear_ratio; 
extern const unsigned int PPR_basemotor;
extern const unsigned long int MOTOR_PPR;


extern const unsigned int PWM_LOW;
extern const unsigned int PWM_HIGH;

extern const int K_SPEED2PWM; //PWM per rps


//enum type for representing the motor state
typedef enum{
  OFF = 0,
  CCW = 1,
  CW = -1
}motorstatedef;

//enum type for representing the motor position
typedef enum{
  LEFT = 1,
  RIGHT = -1,
}motorposition;

// Structure for storing quadrature encoder count
typedef struct {
  volatile unsigned long enc_A_count;
  volatile unsigned long enc_B_count;
}encodercount;


class Motor {
public:
  encodercount enc_count = (encodercount){0,0};
  
	Motor(unsigned int _pin_PWM_FWD, unsigned int _pin_PWM_REV, unsigned int _pin_EN, motorposition _motor_pos, float Kp, float Ki, float Kd, float Kaw);

  void resetEncoder();

  void setMotorSpeedSetpoint(float setpointSpeed);
  
  void setMotorSpeedInput(float setpointSpeed);
  
  void writeMotorPWM();

  void setMotorSpeedMeasurement(float motor_speed);

  void offMotor();

  void setCCW();
  void setCW();
  void incrPWM();
  void decrPWM();

  motorstatedef getMotorState();
  
  motorposition getMotorPosition();

  unsigned int getMotorPWM();

  float getMotorSpeedSetpoint();

  float getMotorSpeedMeasurement();

  float updateMotorSpeed();
 

private:
  unsigned int _pin_PWM_FWD; //pin to which PWM for Forward direction is connected
  unsigned int _pin_PWM_REV; //pin to which PWM for Reverse direction is connected
  unsigned int _pin_EN;  // Enable pin - though the motor has two pins for enable - assuming shorted together

  /*Variable for holding the PWM pin value for the current direction*/
  unsigned int _pin_PWM = 0;

  /*Current state of the motor - OFF, CW, CCW*/
  motorstatedef _current_motor_state = OFF;

  /*Location of the motor -LEFT or RIGHT. Depending upon this motor frame will change*/  
  motorposition _motor_pos = LEFT;   

  /*PWM input given to the motor*/
  unsigned int _pwm_input = 0;

  
  /*Speed setpoint for the motor in motor frame*/
  float _motor_speed_setpoint = 0.0;

  /*Controller input speed after PID calculations*/
  float _motor_speed_input = 0.0;

  /*Measured motor speed*/
  float _measured_motor_speed = 0.0;

  // PID varaibles and parameters

  /*Previous measurment for PID calcualtions*/
  float _measured_motor_speed_prev = 0.0;

  /*Last sampling time for PID*/
  unsigned long _pid_last_sampling_time;

  /*PID integral error data*/
  float _pid_integral = 0.0;

  //PID Gain values
  float _Kp = 0.7;     //0.7 0.4 0.01 lim+-0.4
  float _Ki = 0.0;
  float _Kd = 0.01;
  float _Kaw = 0.1;

  /* Sampling time in seconds*/
  float _T = 0.0;  

  /*Function to reset all the controller variables*/
  void reset_control();
};
#endif