#include "motor.h"

Motor::Motor(unsigned int pin_PWM_FWD, unsigned int pin_PWM_REV, unsigned int pin_EN)
{
  _pin_PWM_FWD = pin_PWM_FWD;
  _pin_PWM_REV = pin_PWM_REV;
  _pin_EN = pin_EN;
  
  pinMode(_pin_PWM_FWD, OUTPUT);
  pinMode(_pin_PWM_REV, OUTPUT);
  pinMode(_pin_EN, OUTPUT);

}
 
void Motor::setMotorSpeed(float motor_speed)
{
  _motor_speed = motor_speed;
}

void Motor::setMotorPWM(unsigned int pwm_input){
  _pwm_input = pwm_input;
}
  
void Motor::writeMotorPWM(unsigned int  pwm_input){
  _pwm_input = pwm_input;
  analogWrite(_pin_PWM, _pwm_input);
}

void Motor::offMotor()
{
  if(_current_motor_state == OFF)
    return;
  _current_motor_state = OFF;
  _pin_PWM = 0;
  analogWrite(_pin_PWM_FWD, 0);
  analogWrite(_pin_PWM_REV, 0);
}

// Direction Functions
void Motor::setCCW() {
  if(_current_motor_state == CCW)
    return;
  else if(_current_motor_state == CW)
    offMotor();

  _current_motor_state = CCW;

  _pin_PWM = _pin_PWM_FWD;
}

void Motor::setCW() {
  if(_current_motor_state == CW)
    return;
  else if(_current_motor_state == CCW)
    offMotor();
  
  _current_motor_state = CW;

  _pin_PWM = _pin_PWM_REV;
} 


//  PWM increment Function 
void Motor::incrPWM()
{
  int pwm_input = _pwm_input + DELTA_PWM;
  pwm_input  = min(pwm_input, 255);
  writeMotorPWM(pwm_input);
}

//  PWM decrement Function 
void Motor::decrPWM()
{
  int pwm_input = _pwm_input + DELTA_PWM;
  pwm_input  = max(pwm_input, 0);
  writeMotorPWM(pwm_input);
}

motorstatedef Motor::getMotorState()
{
  return _current_motor_state;
}

unsigned int Motor::getMotorPWM()
{
  return _pwm_input;
}

float Motor::getMotorSpeed()
{
  return _motor_speed;
}


