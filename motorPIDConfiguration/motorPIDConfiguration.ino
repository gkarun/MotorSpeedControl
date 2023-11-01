#define USE_TIMER_5 true


#include "TimerInterrupt.h"        //https://github.com/khoih-prog/TimerInterrupt
#include "ISR_Timer.h"              //https://github.com/khoih-prog/TimerInterrupt

const unsigned long int TIMER_INTERVAL_MS=50L;
const float gear_ratio = 2800/60; 
const unsigned int PPR_basemotor = 500*4;
const unsigned long int MOTOR_PPR = PPR_basemotor*gear_ratio;
const double OMEGA_MULTIPLIER = MOTOR_PPR*TIMER_INTERVAL_MS/1000; // rotation per second calculation inverse

const int K_SPEED2PWM=255; //PWM per rps
const float SPEED_DEFAULT_SETPOINT = .6; // rotation per second


int M1_PWM_setpoint = 0;

// PID varaibles and parameters

const double Kp = 0.8;     //0.7 0.4 0.01 lim+-0.4
const double Ki = 0.0;
const double Kd = 0.015;  
const double Kaw = 0.1;                                                                                                                                                                                                                 

double error = 0.0;
double prop = 0.0;
double integral = 0.0;
double differ = 0.0;
double update = 0.0;
double T = 0.0; // Sampling time in seconds 

double prev_measurement = 0.0;

double min_input_limit = -0.3;
double max_input_limit = 0.3;

unsigned long last_sampling_time;

//PID Parameters end!

// Time reference for plotting data
unsigned long old_time = micros();

typedef enum{
  STOP,
  FORWARD,
  REVERSE
}motorstatedef;

typedef union {
  float motor_speed;
  byte bin[4];
} speedvar;

typedef union {
  float time;
  byte bin[4];
} time_in_secs;

time_in_secs t;

motorstatedef current_motor_state = STOP;

// ---------- M1 Variables START------------------
// Motor1 connections
const int M1_PWM_FWD = 2; //
const int M1_PWM_REV = 3; //
const int M1_EN_FWD = 40;
const int M1_EN_REV = 40;

// Variable for switching forward and reverse 
unsigned int M1_PWM = 0;

// Variables for encoder data count
unsigned int enc_A_M1_count = 0;
unsigned int enc_B_M1_count = 0;


//Variable for storing omega of M1
speedvar M1_omega;
speedvar speed_setpoint ;

// ---------- M1 Variables END------------------



void setup() {
  //  Starting of serial communication 
  Serial.begin(115200);  


  // Init timer ITimer3
  ITimer5.init();

  //ITimer5.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);
  // Interval in unsigned long millisecs
  if (ITimer5.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
    Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");
  
  
  // ---------- Motor 1 settings START-------------- 
  // Motor 1 pin setup
  pinMode(M1_PWM_FWD, OUTPUT);
  pinMode(M1_PWM_REV, OUTPUT);
  pinMode(M1_EN_FWD, OUTPUT);
  pinMode(M1_EN_REV, OUTPUT);
  
  // Attaching interrupt to encoder pins 
  attachInterrupt(digitalPinToInterrupt(20),encoderA_M1,CHANGE); 
  attachInterrupt(digitalPinToInterrupt(21),encoderA_M1,CHANGE); 

  digitalWrite(M1_EN_FWD, HIGH);
  digitalWrite(M1_EN_REV, HIGH);

  // ---------- Motor 1 setting ENDS -------------- 
  
}
 long time_start = millis();
void loop() {
 
  /*long time_end = millis();
  Serial.print("TIME FOR LOOP in millis:  ------- >>>>  ");
  Serial.println(time_end - time_start);
  time_start = time_end;
  */
    if (Serial.available()) {                
      char inChar = (char)Serial.read();
      Serial.println("DEBUG ARDUINO: Data Received : ");
      Serial.println(inChar);
      Serial.read();

    switch(inChar){                            // Switch Cases for Motor Direction 
      case '0':
        stopMotor();
        //Serial.println("DEBUG ARDUINO: stopMotor");
        break;
      case '8':
        forward();
        //Serial.println("DEBUG ARDUINO: forward");
        break;
      case '2':
        reverse();
       //Serial.println("DEBUG ARDUINO: reverse");
        break;
      case '4':
        decrSpeed();
        //Serial.println("DEBUG ARDUINO: decrSpeed");
        break;
      case '6':
        incrSpeed();
        //Serial.println("DEBUG ARDUINO: incrSpeed");
        break;
      default: 
        stopMotor();
       Serial.print(inChar);
        //Serial.println("DEBUG ARDUINO:  ------- stopMotor ");
        break;   
    } 


  }

  if(M1_PWM)
  {
    double delta_speed = PID_calculation(speed_setpoint.motor_speed, M1_omega.motor_speed);
    if(delta_speed != 0)
    {
      M1_PWM_setpoint += K_SPEED2PWM*delta_speed;
      // Using Contrain to limit the PWM input to the Motors  
      M1_PWM_setpoint = constrain(M1_PWM_setpoint, 0, 255);
      analogWrite(M1_PWM, M1_PWM_setpoint);
      Serial.print(" PWM: ");
      Serial.println(M1_PWM_setpoint);
      //Serial.print("DEBUG ARDUINO: delta speed: ");
      //Serial.println(delta_speed);
    }
  }

  send_serial_log_data();

}

double PID_calculation(double setpoint, double measurement){
  error = setpoint - measurement;

  unsigned long  now = millis();
  T = ((double)(now -last_sampling_time))/1000;
  if(T < 0.1){
    return 0.0f;
  }
  
  
  prop = Kp*error;
  


  if(abs(Ki) > 0.0)
  { 
    //Serial.print("DEBUG ARDUINO: before integral update --- ");
    //Serial.println(integral);
    integral = integral + Ki*T*error;
  }

  //Serial.print("DEBUG ARDUINO: After integral update --- ");
  //Serial.println(integral);

  // Avoiding derivative kick by removing change in setpoint from the derivative calc
  if(abs(Kd) > 0.0)
    differ = -Kd*(measurement-prev_measurement)*(1/T);

  update = prop + integral  + differ;


  // Anti integral windup and output saturation
  if(update > max_input_limit){
    //Serial.print("DEBUG ARDUINO: Limiting MAX ");
    if(Ki != 0)
      integral = integral - Kaw*(update - max_input_limit);
    update = max_input_limit;
  } else if (update < min_input_limit){
    //Serial.print("DEBUG ARDUINO: Limiting MIN ");
    if(Ki != 0)
      integral = integral + Kaw*(min_input_limit - update);
    update = min_input_limit;
  }

  prev_measurement = measurement;
  last_sampling_time = now;
  
  Serial.print("DEBUG ARDUINO: error: ");
  Serial.print(error);
  Serial.print("  T: ");
  Serial.print(T);
  Serial.print(" P: ");
  Serial.print(prop);
  Serial.print(" I: ");
  Serial.print(integral);
  Serial.print(" D: ");
  Serial.print(differ);
  Serial.print(" update: ");
  Serial.print(update);
  
  return update;
  
}

void reset_control(){
  error = 0;
  integral = 0;
  prev_measurement = 0;
  last_sampling_time = millis();
  noInterrupts();
  enc_A_M1_count = 0;
  M1_omega.motor_speed = 0;
  interrupts();  
}

void send_serial_log_data()
{

    long elapsed_time = 0;
    unsigned long current_time = 0;
    int direction_sign = 1;

    current_time = micros();
    //Serial.println(current_time);
    //Serial.println(old_time);
    elapsed_time = current_time - old_time;
    //Serial.println(elapsed_time);
    
    if(current_motor_state == REVERSE)
      direction_sign = -1;
    speedvar M1_w = M1_omega;
    speedvar M1_sp = speed_setpoint;

    M1_w.motor_speed = direction_sign*M1_w.motor_speed;
    M1_sp.motor_speed = direction_sign*M1_sp.motor_speed;
    Serial.print("*");
    t.time = (double)elapsed_time/1000000.0f;
    Serial.write(t.bin,4);
    Serial.write(M1_w.bin,4);
    Serial.write(M1_sp.bin,4);
}

// Direction Functions
void forward() {
   if(current_motor_state == FORWARD)
    return;
  else if(current_motor_state == REVERSE)
  {
    stopMotor();
    reset_control();
  }
  current_motor_state = FORWARD;
  speed_setpoint.motor_speed = SPEED_DEFAULT_SETPOINT;
  M1_PWM = M1_PWM_FWD;
  last_sampling_time = millis();

 }

void reverse() {
 if(current_motor_state == REVERSE)
    return;
  else if(current_motor_state == FORWARD)
  {
    stopMotor();
    reset_control();
  }

  current_motor_state = REVERSE;
  speed_setpoint.motor_speed = SPEED_DEFAULT_SETPOINT;
  M1_PWM = M1_PWM_REV;
  last_sampling_time = millis();
} 



void stopMotor()
{
  if(current_motor_state == STOP)
    return;
  current_motor_state = STOP;
  M1_PWM = 0;
  speed_setpoint.motor_speed = 0;
  M1_PWM_setpoint = 0;
  analogWrite(M1_PWM_FWD, M1_PWM_setpoint);
  analogWrite(M1_PWM_REV, M1_PWM_setpoint);
}


//  Speed increament Function | 0.2 RPS/increament
void incrSpeed()
{
  speed_setpoint.motor_speed += 0.05;
  speed_setpoint.motor_speed = min(1, speed_setpoint.motor_speed);

  //Serial.print("DEBUG ARDUINO: incr speed: ");
  Serial.println(speed_setpoint.motor_speed);
}

//  Speed decreament Function | 0.2 RPS/decreament
void decrSpeed()
{
  speed_setpoint.motor_speed -= 0.05;
  speed_setpoint.motor_speed = max(0, speed_setpoint.motor_speed);

  //Serial.print("DEBUG ARDUINO: decr speed --- ");
  Serial.println(speed_setpoint.motor_speed);
}

void encoderA_M1(){
  enc_A_M1_count++;
  //Serial.print("DEBUG ARDUINO: inside ISR -------------------------- ");

}
void encoderB_M1(){
  enc_B_M1_count++;

}

void TimerHandler()
{
  unsigned long int encoder_count = enc_A_M1_count;
  noInterrupts();
  enc_A_M1_count = 0;
  interrupts();

  M1_omega.motor_speed = ((double)encoder_count)/OMEGA_MULTIPLIER; 
  //Serial.print("DEBUG ARDUINO: Speed Calculation - --- ");
  //Serial.println(M1_omega.motor_speed);
}