#include "Arduino.h"
#include "motor.h"



const float gear_ratio = 2800/60; 
const unsigned int PPR_basemotor = 500;
const unsigned long int MOTOR_PPR = 93132/4; //As per robokits doc. Avoiding PPR_basemotor*gear_ratio  calculation as these numbers might be approximate;


const unsigned int PWM_LOW = 0;
const unsigned int PWM_HIGH = 150; //175 is the safest highest value as beyond this ISR is blocking the main loop and erratic behaviour happens

const int K_SPEED2PWM=255/TWO_PI; //PWM per rps

//Change in pwm while incrementing or decrementing the speed
const unsigned int DELTA_PWM = 5;


//Change in pwm while incrementing or decrementing the speed
const  float PID_TIME_INTERVAL = 0.1;

const float MIN_INPUT_LIMIT = -0.3;
const float MAX_INPUT_LIMIT = 0.3;
//PID Parameters end!

Motor::Motor(unsigned int pin_PWM_FWD, unsigned int pin_PWM_REV, unsigned int pin_EN, motorposition pos, float Kp, float Ki, float Kd, float Kaw)
{
  _motor_pos = pos;
  _pin_PWM_FWD = pin_PWM_FWD;
  _pin_PWM_REV = pin_PWM_REV;
  _pin_EN = pin_EN;
  
  pinMode(_pin_PWM_FWD, OUTPUT);
  pinMode(_pin_PWM_REV, OUTPUT);
  pinMode(_pin_EN, OUTPUT);

  _Kp  = Kp;
  _Ki  = Ki;
  _Kd  = Kd;
  _Kaw = Kaw;
  enc_count = (encodercount){0,0};

}

void Motor::setMotorSpeedMeasurement(float motor_speed)
{
  _measured_motor_speed = _current_motor_state * motor_speed;
}


void Motor::setMotorSpeedSetpoint(float setpointSpeed)
{
  _motor_speed_setpoint = _motor_pos*setpointSpeed;
}
  
void Motor::setMotorSpeedInput(float update)
{
  _motor_speed_input += update ;

#ifdef DEBUG
      Serial.print("----->  DEBUG ARDUINO: motor speed input : ");
      Serial.println(_motor_speed_input);
#endif

}
  
void Motor::writeMotorPWM()
{
  /*Setting direction*/
  if (_motor_speed_input == 0) {
    offMotor();
    return;
  }else if (_motor_speed_input > 0)
  {
    setCCW();
  }else  {
    setCW();
  }

  int pwm_input = K_SPEED2PWM*abs(_motor_speed_input);
  _pwm_input = constrain(pwm_input, PWM_LOW, PWM_HIGH);
  analogWrite(_pin_PWM, _pwm_input);
  //analogWrite(_pin_PWM, 255);

//#ifdef DEBUG
  Serial.print(_pwm_input);
  Serial.print("  ");
//#endif
}

void Motor::offMotor()
{
  if(_current_motor_state == OFF)
    return;
  
  _current_motor_state = OFF;
  _pin_PWM = 0;
  analogWrite(_pin_PWM_FWD, 0);
  analogWrite(_pin_PWM_REV, 0);
  resetEncoder();
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
  _pwm_input  = min(pwm_input, PWM_HIGH);
  writeMotorPWM();
}

//  PWM decrement Function 
void Motor::decrPWM()
{
  int pwm_input = _pwm_input + DELTA_PWM;
  _pwm_input  = max(pwm_input, PWM_LOW);
  writeMotorPWM();
}


float Motor::getMotorSpeedSetpoint()
{
  return _motor_speed_setpoint;
}

motorstatedef Motor::getMotorState()
{
  return _current_motor_state;
}

motorposition Motor::getMotorPosition()
{
  return _motor_pos;
}

unsigned int Motor::getMotorPWM()
{
  return _pwm_input;
}

float Motor::getMotorSpeedMeasurement()
{
  return  _measured_motor_speed;
}

void Motor::resetEncoder()
{
  noInterrupts();
  enc_count = (encodercount){0,0};
  interrupts();
}

float Motor::updateMotorSpeed(){
  float error = 0.0;
  float prop = 0.0;
  float differ = 0.0;
  float update = 0.0;
  float measurement = 0.0;

  unsigned long  now = millis();
  _T = ((float)(now -_pid_last_sampling_time))/1000;
  if(_T < PID_TIME_INTERVAL){
    return update;
  }

  if(_motor_speed_setpoint == 0) 
  {
    reset_control();
    offMotor();
    return update;

  }
  else if( sgn(_motor_speed_setpoint) != _current_motor_state) 
  {
    reset_control();
  }
   else
  {
    measurement = _measured_motor_speed;
  }

  error = _motor_speed_setpoint - measurement;

  prop = _Kp*error;
  
  if(abs(_Ki) > 0.0)
  { 
    //Serial.print("DEBUG ARDUINO: before integral update --- ");
    //Serial.println(integral);
    _pid_integral = _pid_integral + _Ki*_T*error;
  }

  //Serial.print("DEBUG ARDUINO: After integral update --- ");
  //Serial.println(integral);

  // Avoiding derivative kick by removing change in setpoint from the derivative calc
  if(abs(_Kd) > 0.0)
    differ = -_Kd*(measurement - _measured_motor_speed_prev)*(1/_T);

  update = prop + _pid_integral  + differ;


  // Anti integral windup and output saturation
  if(update > MAX_INPUT_LIMIT){
    //Serial.print("DEBUG ARDUINO: Limiting MAX ");
    if(_Ki != 0)
      _pid_integral = _pid_integral - _Kaw*(update - MAX_INPUT_LIMIT);
    update = MAX_INPUT_LIMIT;
  } else if (update < MIN_INPUT_LIMIT){
    //Serial.print("DEBUG ARDUINO: Limiting MIN ");
    if(_Ki != 0)
      _pid_integral = _pid_integral + _Kaw*(MIN_INPUT_LIMIT - update);
    update = MIN_INPUT_LIMIT;
  }

  _measured_motor_speed_prev = measurement;
  _pid_last_sampling_time = now;
  
  #ifdef DEBUG
    Serial.print("DEBUG ARDUINO: error: ");
    Serial.print(error);
    Serial.print("  T: ");
    Serial.print(T);
    Serial.print(" P: ");
    Serial.print(prop);
    Serial.print(" I: ");
    Serial.print(_pid_integral);
    Serial.print(" D: ");
    Serial.print(differ);
    Serial.print(" update: ");
    Serial.print(update);
  #endif

  setMotorSpeedInput(update);
  
  return update;
}

void Motor::reset_control(){
  _pid_integral = 0;
  _measured_motor_speed_prev = 0;
  _motor_speed_input = 0.0;
}
