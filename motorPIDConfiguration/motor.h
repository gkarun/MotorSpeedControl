#ifndef Motor_h
#define Motor_h

#include "Arduino.h" 

const float gear_ratio = 2800/60; 
const unsigned int PPR_basemotor = 500;
const unsigned long int MOTOR_PPR = PPR_basemotor*gear_ratio;

const int K_SPEED2PWM=255; //PWM per rps

//Change in pwm while incrementing or decrementing the speed
const unsigned int DELTA_PWM = 5;


//enum type for representing the motor state
typedef enum{
  OFF,
  CCW,
  CW
}motorstatedef;

// Structure for storing quadrature encoder count
typedef struct {
  unsigned int enc_A_count;
  unsigned int enc_B_count;
}encodercount;

class Motor {
public:
	Motor(unsigned int _pin_PWM_FWD, unsigned int _pin_PWM_REV, unsigned int _pin_EN);

  /* ISR for both the encoders*/
  void incrEncoderA();
  void incrEncoderB();

  encodercount getEncoderCounts();

  void resetEncoderCounts();

  void setMotorPWM(unsigned int pwm_input);
  
  void writeMotorPWM(unsigned int  pwm_input);

  void setMotorSpeed(float motor_speed);

  void offMotor();

  void setCCW();
  void setCW();
  void incrPWM();
  void decrPWM();

  motorstatedef getMotorState();
  
  unsigned int getMotorPWM();

  float getMotorSpeed();
 

private:
  unsigned int _pin_PWM_FWD; //pin to which PWM for Forward direction is connected
  unsigned int _pin_PWM_REV; //pin to which PWM for Reverse direction is connected
  unsigned int _pin_EN;  // Enable pin - though the motor has two pins for enable - assuming shorted together

  unsigned int _pin_encA; // Pin to which Encoder A pulses are connected
  unsigned int _pin_encB; // Pin to which Encoder B pulses are connected
  
  
  
  motorstatedef _current_motor_state = OFF;

  unsigned int _pwm_input = 0;

  unsigned int _pin_PWM = 0;

  float _motor_speed;

};
#endif