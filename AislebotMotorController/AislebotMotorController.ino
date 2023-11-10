#include "motor.h"
#define USE_TIMER_5 true
#define DEBUG

#include "TimerInterrupt.h"        //https://github.com/khoih-prog/TimerInterrupt
#include "ISR_Timer.h"              //https://github.com/khoih-prog/TimerInterrupt

bool CheckEncoderFeedback = true;

const unsigned long int TIMER_INTERVAL_MS=50L;
const float OMEGA_MULTIPLIER = (1000*2*PI)/((float)MOTOR_PPR*TIMER_INTERVAL_MS); // multiplier for converting encoder reading in TIMER INTERVAL to radian per second

const float SPEED_DEFAULT_SETPOINT = .6; // rotation per second


const float WHEEL_RADIUS = 76.2;

//Data structure for sending data as binary to main controller
typedef union {
  float data;
  byte bin[4];
} float_bin_union;


// ---------------------- Motor Pins declaration START --------------------------
// ---------------- Motor and encoder pins connected to arduino -----------------
// ------------------------ Front Right M1 Variables ----------------------------
// Motor1 connections
const int M_PWM_FWD[4] = {2,4,6,8}; //
const int M_PWM_REV[4] = {3,5,7,9}; //
const int M_encA[4] = {18,19,20,21};
const int M_encB[4] = {31,33,35,37};
const motorposition M_ALIGN[4] = {RIGHT,LEFT, RIGHT, LEFT};

const float Kp[4] = {0.7,0.7,0.7,0.9};
const float Ki[4] = {0.0,0.0,0.0,0.0};
const float Kd[4] = {0.01,0.01,0.01,0.005};
const float Kaw[4] = {0.0,0.0,0.0,0.1};


const int MOTOR_EN = 40; // All motor are enabled from a single pin

// ---------------------- Motor Pins declaration END --------------------------

Motor M[4] = {

  Motor(M_PWM_FWD[0], M_PWM_REV[0], MOTOR_EN, M_ALIGN[0], Kp[0], Ki[0], Kd[0], Kaw[0] ),
  Motor(M_PWM_FWD[1], M_PWM_REV[1], MOTOR_EN, M_ALIGN[1], Kp[1], Ki[1], Kd[1], Kaw[1] ),
  Motor(M_PWM_FWD[2], M_PWM_REV[2], MOTOR_EN, M_ALIGN[2], Kp[2], Ki[2], Kd[2], Kaw[2] ),
  Motor(M_PWM_FWD[3], M_PWM_REV[3], MOTOR_EN, M_ALIGN[3], Kp[3], Ki[3], Kd[3], Kaw[3] )

};



//Variable for storing the spped setpoints
// u - speed in x direction or forward direction  of robot
// v - speed in y direction or towards the left of robot
// w - roatational speed
float_bin_union u_setpoint_bf;
float_bin_union v_setpoint_bf;
float_bin_union w_setpoint_bf;


float_bin_union omega_setpoint_bf[4];


float kinematics[][3] = {{1.0, -1.0, -246.0},
                         {1.0,  1.0,  176.0},
                         {1.0,  1.0, -176.0},
                         {1.0, -1.0,  246.0}};  

unsigned long old_time = micros();


void setup() {
  //  Starting of serial communication 
  Serial.begin(115200); 

  //Enabling the motors
  digitalWrite(MOTOR_EN, HIGH);
    
  //If encoder connected, attaching the encoder interrupts to the ISR
  if(CheckEncoderFeedback)
  {
      // Init timer ITimer3
    ITimer5.init();

    //ITimer5.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);
    // Interval in unsigned long millisecs
    if (ITimer5.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler))
      Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
    else
      Serial.println("Can't set ITimer. Select another freq. or timer");
    
    
    attachInterrupt(digitalPinToInterrupt(M_encA[0]),M1_incrEncoderA,RISING);
    //attachInterrupt(digitalPinToInterrupt(M_encB[0]),M1_incrEncoderB,CHANGE);
    attachInterrupt(digitalPinToInterrupt(M_encA[1]),M2_incrEncoderA,RISING);
    //attachInterrupt(digitalPinToInterrupt(M_encB[1]),M2_incrEncoderB,CHANGE);
    attachInterrupt(digitalPinToInterrupt(M_encA[2]),M3_incrEncoderA,RISING);
    //attachInterrupt(digitalPinToInterrupt(M_encB[2]),M3_incrEncoderB,CHANGE);
    attachInterrupt(digitalPinToInterrupt(M_encA[3]),M4_incrEncoderA,RISING);
    //attachInterrupt(digitalPinToInterrupt(M_encB[3]),M4_incrEncoderB,CHANGE);
  }
  
}
long time_start = millis();

void loop() {

  long time_end = millis();
  //Serial.print("TIME FOR LOOP in millis:  ------- >>>>  ");
  //Serial.println(time_end - time_start);
  time_start = time_end;
  

    if (Serial.available()>=14) {
                      
      char inChar = (char)Serial.read();
      //Serial.println("DEBUG ARDUINO: Data Received : ");
      //Serial.println(inChar);

      if(inChar == '<')
      {
        //Serial.println("DEBUG : Start of serial message");
        
        Serial.readBytes((byte*)&u_setpoint_bf, 4);
        Serial.readBytes((byte*)&v_setpoint_bf, 4);
        Serial.readBytes((byte*)&w_setpoint_bf, 4);
      }
      inChar = (char)Serial.read();
      /*if(inChar == '>'){
        Serial.print("DEBUG: U = ");
        Serial.println(u_setpoint_bf.data);
        Serial.print("DEBUG: V = ");
        Serial.println(v_setpoint_bf.data);
        Serial.print("DEBUG: W = ");
        Serial.println(w_setpoint_bf.data);             
        Serial.println("DEBUG : End of serial message");
      }else {
        return;
      }*/
    } 
  
  //Serial.print("Setpoint : ");
      
  for (int i=0; i<4; i++)
  {
    omega_setpoint_bf[i].data = (kinematics[i][0]*u_setpoint_bf.data + kinematics[i][1]*v_setpoint_bf.data + kinematics[i][2]*w_setpoint_bf.data)/WHEEL_RADIUS;
//#ifdef DEBUG
      
      
//#endif
    M[i].setMotorSpeedSetpoint(omega_setpoint_bf[i].data );

    //Serial.print(M[i].getMotorSpeedSetpoint(),5);
    //Serial.print(" ");

  }
  //Serial.println("");
  //Serial.print("DEBUG ARDUINO: delta speed: ");
  for (int i=0; i<4; i++)
  {
    if(CheckEncoderFeedback)
    {
      float delta_speed = M[i].updateMotorSpeed();
#ifdef DEBUG
      
      //Serial.print(delta_speed*M[i].getMotorPosition());
      //Serial.print(" ");
#endif 
    }else {
      M[i].setMotorSpeedInput(M[i].getMotorSpeedSetpoint());
    }
  }
  //Serial.println("");
  //Keep this loop seperate for a better synchronised update
  Serial.print("DEBUG ARDUINO: PWM : ");
  for (int i=0; i<4; i++)
  {
    
    M[i].writeMotorPWM();
  }
  Serial.println("");
  send_serial_log_data();
}

void send_serial_log_data()
{
    //Serial.println("DEBUG ARDUINO: send serial log data start ");
    float_bin_union speed_val;
    float_bin_union time_val;
    long elapsed_time = 0;
    unsigned long current_time = 0;
    int direction_sign = 1;

    current_time = micros();
    //Serial.println(current_time);
    //Serial.println(old_time);
    elapsed_time = current_time - old_time;
    time_val.data = (float)elapsed_time/1000000.0f;

    Serial.print("*");
    Serial.write(time_val.bin,4);
    for(int i = 0; i<4; i++)
    {
      speed_val.data = M[i].getMotorPosition()*M[i].getMotorSpeedMeasurement();
      Serial.write(speed_val.bin,4);
      Serial.write(omega_setpoint_bf[i].bin,4);
    }  


    //Serial.print("Motor Status :  ");
    /*for(int i = 0; i<4; i++)
    {
      //speed_val.data = M[i].getMotorPosition()*M[i].getMotorSpeedMeasurement();
      
      Serial.print(M[i].getMotorSpeedMeasurement());    
      Serial.print("    ");
    }
    Serial.println(" ");*/
}

void stopRobot()
{
  //Disabling motors
  digitalWrite(MOTOR_EN, LOW);
}



void TimerHandler()
{
  unsigned long MEncCountA[4];
  for (int i=0; i<4; i++) {
    MEncCountA[i] = M[i].enc_count.enc_A_count;
  }
  
  resetEncoderCounts();
  
  for (int i=0; i<4; i++) {
    M[i].setMotorSpeedMeasurement(((float)MEncCountA[i])*OMEGA_MULTIPLIER);  
    //Serial.println(MEncCountA[i]);
    //Serial.println(((float)MEncCountA[i])*OMEGA_MULTIPLIER);
  }
}

// Interrupt Service routines

void M1_incrEncoderA()
{
  M[0].enc_count.enc_A_count++;
}
void M1_incrEncoderB()
{
  M[0].enc_count.enc_A_count++;
}
void M2_incrEncoderA()
{
  M[1].enc_count.enc_A_count++;
}
void M2_incrEncoderB()
{
  M[1].enc_count.enc_A_count++;
}
void M3_incrEncoderA()
{
  M[2].enc_count.enc_A_count++;
}
void M3_incrEncoderB()
{
  M[2].enc_count.enc_A_count++;
}
void M4_incrEncoderA()
{
  M[3].enc_count.enc_A_count++;
}
void M4_incrEncoderB()
{
  M[3].enc_count.enc_A_count++;
}

//Reset all the encoder counts
void resetEncoderCounts()
{
  noInterrupts();
  for (int i=0; i<4; i++) {
    M[i].enc_count =(encodercount){0,0};
  }
  interrupts();
}


