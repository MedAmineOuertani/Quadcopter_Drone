///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  RC Module Libraries
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <SPI.h>             // Arduino Serial Peripheral Interface protocol library
#include "nRF24L01.h"
#include "RF24.h"            // RC transceiver module libraries

//  Defines
#define CE   4
#define CSN  5
#define SCK  18
#define MOSI 23
#define MISO 19
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BLDC Motor\ESC Librarie
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ESP32_Servo.h>
//  Defines
#define ESC_0    33        // PWM pin to connect with corresponding ESC
#define ESC_1    12        // These pins control the rotation speed of the BLDC motors
#define ESC_2    13
#define ESC_3    32
// minimum and maximum PWM pulse width
#define MIN_THROTTLE   1000      // you can see it as minimum throttle that can be applied to motors    
#define MAX_THROTTLE    1700      // maximum throttle that can be applied to motors

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MPU9250 Module Libraries
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <MPU9250.h>
#include <Wire.h>

// Defines
MPU9250 mpu;
int16_t rollAvr = 0;
int16_t pitchAvr = 0;
int16_t yawAvr = 0;
int16_t Roll=0, Pitch=0, Yaw=0;
bool calibrate = true;
void inline MPU_Angles_Avr(int16_t &rollavr, int16_t &pitchavr, int16_t &yawavr);
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID gain settings for pich , roll and yaw
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PID_PITCH_kp  0
#define PID_PITCH_ki  0
#define PID_PITCH_kd  0

#define PID_ROLL_kp   PID_PITCH_kp
#define PID_ROLL_kd   PID_PITCH_kd
#define PID_ROLL_ki   PID_PITCH_ki

#define PID_YAW_kp    0
#define PID_YAW_kd    0
#define PID_YAW_ki    0

float previous_pitch_error = .0;
float previous_roll_error = .0;

int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch);
int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll);

void    inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output, const int16_t roll_pid_output, const int16_t yaw_pid_output);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Uncomment when needed
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define DEBUG        // for adding and removing debug code 
//#define DEBUG_MPU
//#define DEBUG_NRF
//#define DEBUG_MOTORS_SPEED
//#define DEBUG_PID_VALUES



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Servo MOTOR_0;
Servo MOTOR_1;
Servo MOTOR_2;
Servo MOTOR_3;

RF24 receiver(CE, CSN, SCK, MISO, MOSI);  //check link for class methods:https://maniacbug.github.io/RF24/classRF24.html

const byte add[6] = "00001";              //IMPORTANT: The same as in the transmitter

float values_received[4];

#define led 2

float elapsedTime, now, timePrev;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void){
  pinMode(led, OUTPUT);
#ifdef DEBUG
  Serial.begin(9600);
#endif
  digitalWrite(led, HIGH);
  // setting-up the motors //
  MOTOR_0.attach(ESC_0, MIN_THROTTLE, MAX_THROTTLE);
  MOTOR_1.attach(ESC_1, MIN_THROTTLE, MAX_THROTTLE);
  MOTOR_2.attach(ESC_2, MIN_THROTTLE, MAX_THROTTLE);
  MOTOR_3.attach(ESC_3, MIN_THROTTLE, MAX_THROTTLE);
  // calibrating the ESCs
  delay(500);
  MOTOR_0.writeMicroseconds(MAX_THROTTLE);
  MOTOR_1.writeMicroseconds(MAX_THROTTLE);
  MOTOR_2.writeMicroseconds(MAX_THROTTLE);
  MOTOR_3.writeMicroseconds(MAX_THROTTLE);
  delay(5000);
  MOTOR_0.writeMicroseconds(MIN_THROTTLE);
  MOTOR_1.writeMicroseconds(MIN_THROTTLE);
  MOTOR_2.writeMicroseconds(MIN_THROTTLE);
  MOTOR_3.writeMicroseconds(MIN_THROTTLE);
  delay(3000);

  // setting-up nRF module //
  receiver.begin();       // Begin operation of the chip.
  receiver.setChannel(2); // Which RF channel to communicate on, 0-127
  receiver.setPayloadSize(16);
  receiver.setDataRate(RF24_250KBPS);
  receiver.openReadingPipe(1, add); // Open the pipe number 1 (0-5) for reading .
  receiver.setPALevel(RF24_PA_MIN); // Set Power Amplifier (PA) level to one of four levels (RF24_PA_MIN, RF24_PA_LOW, RF24_PA_MED, RF24_PA_HIGH).
  receiver.startListening();        // Set the radio comunication to receiver mode

  // setting-up the MPU //
  Wire.begin();
   
  if (!mpu.setup(0x68))
  {
#ifdef DEBUG_MPU
    Serial.println("Failed to initialize MPU9250.");
#endif

    while (1)
    {
    }
  }
  

  mpu.calibrateAccelGyro();
  MPU_Angles_Avr(rollAvr, pitchAvr, yawAvr);

  digitalWrite(led, LOW);

  now = millis(); //Start counting time in milliseconds
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
 timePrev = now;  // the previous time is stored before the actual time read
    now = millis();  // actual time read
    elapsedTime = (now - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/
//nrf
if (receiver.available()){  
    values_received[0] = 0; //throtlle
    values_received[1] = 0; //kp
    values_received[2] = 0; //ki
    values_received[3] = 0; //kd
     
     receiver.read(&values_received, sizeof(float)*4);
     #ifdef DEBUG_NRF
      Serial.print(" throttle : ");
  Serial.print(int16_t(values_received[0]));
  Serial.print(" kp : ");
  Serial.print(values_received[1]);
  Serial.print(" ki : ");
  Serial.print(values_received[2]);
  Serial.print(" kd : ");
  Serial.print(int16_t(values_received[3]));
  Serial.print("\n");
      
     #endif
     delay(70);
}
else{
//secure landing
    #ifdef DEBUG_NRF
     Serial.print("no radio available \n");
    #endif
     MOTOR_0.writeMicroseconds(MIN_THROTTLE);
     MOTOR_1.writeMicroseconds(MIN_THROTTLE);
     MOTOR_2.writeMicroseconds(MIN_THROTTLE);
     MOTOR_3.writeMicroseconds(MIN_THROTTLE);
}
//MPU


Measured_Roll_Pitch_Yaw(Roll, Pitch, Yaw);

// this code is for tunig only the pitch PID and the set point is zero.
// and you can change it later for tunig roll and yaw PIDs.
int16_t PitchPIDOutput = ExecutePitchPID(0, Pitch);
int16_t RollPIDOutput   = ExecutePitchPID(0, 0);//(0, Roll);
//int16_t YawPIDOutput    = ExecuteYawPID(values_received[3], measured_yaw);
#ifdef DEBUG_PID_VALUES
Serial.print("pitch PID output: ");
Serial.print(PitchPIDOutput);
Serial.print(" roll PID output: ");
Serial.print(RollPIDOutput);
Serial.print("\n");
#endif
UpdateMotorsValues(int16_t(values_received[0]), PitchPIDOutput, RollPIDOutput, 0);
}






// this function calculates the average of each angle when starting the flight controller
void inline MPU_Angles_Avr(int16_t & rollavr, int16_t & pitchavr, int16_t & yawavr){
    #ifdef DEBUG_MPU
        Serial.print("Calibration start");
    #endif
  for (int i = 0; i < 2000; i++)
  {
    mpu.update();
    float x_angle = mpu.getRoll();
    float y_angle = mpu.getPitch();
    float z_angle = mpu.getYaw();
    rollavr += (x_angle/2000);
    pitchavr += (y_angle/2000);
    yawavr += (z_angle/2000);
    delay(1);
  }
  #ifdef DEBUG_MPU
    Serial.print("Calibration done");
  #endif
}
//this function measures the roll, pitch and yaw
void inline Measured_Roll_Pitch_Yaw(int16_t & Roll, int16_t & Pitch, int16_t & Yaw){
    if (mpu.update()) {
        Roll = mpu.getRoll() - rollAvr - 7;
        Pitch = mpu.getPitch() - pitchAvr -9;
        Yaw = mpu.getYaw() - yawAvr;
    #ifdef DEBUG_MPU
        Serial.print("Roll: ");
        Serial.print(Roll);
        Serial.print(" Pitch: ");
        Serial.print(Pitch);
        Serial.print(" Yaw: ");
        Serial.print(Yaw);
        Serial.print("\n");
#endif
    }
}



int16_t inline ExecutePitchPID(const int16_t& pitch_set_point, const int16_t& measured_pitch) {
  float error;
  float i;
  float integral = .0;

  float proportional;
  float derivative;
  
  error = static_cast<float>(pitch_set_point - measured_pitch);
  //if(error<=5 && error>=-5)
       // error = 0;
  proportional = values_received[1] * error;
  derivative = (values_received[3]) * (error - (previous_pitch_error))/elapsedTime;
 // if(values_received[4]-15 <error <values_received[4]+15){
  i += ((previous_pitch_error + error)/2)*elapsedTime;
  integral = i * values_received[2];
 // }
  previous_pitch_error = error;
  
  return static_cast<int16_t>(proportional + integral + derivative);
}

int16_t inline ExecuteRollPID(const int16_t& roll_set_point, const int16_t& measured_roll) {
  float error;
  float i;
  float integral = .0;

  float proportional;
  float derivative;
  
  error = roll_set_point - measured_roll;
  //if(error<=10 && error>=-10)
        //error = 0;
  proportional = values_received[1] * error;
  derivative = (values_received[3]) * (error - (previous_roll_error))/elapsedTime;
  i += ((previous_roll_error + error)/2)*elapsedTime;
  integral = i * values_received[2];
  previous_roll_error = error;
  
  return static_cast<int16_t>(proportional + integral + derivative);
}
void inline UpdateMotorsValues( const int16_t throttle, const int16_t pitch_pid_output,
                                const int16_t roll_pid_output, const int16_t yaw_pid_output) {
  int16_t m0 = throttle - pitch_pid_output - roll_pid_output + yaw_pid_output;
  int16_t m1 = throttle - pitch_pid_output + roll_pid_output - yaw_pid_output;
  int16_t m2 = throttle + pitch_pid_output - roll_pid_output - yaw_pid_output;
  int16_t m3 = throttle + pitch_pid_output + roll_pid_output + yaw_pid_output;

  m0 = constrain(m0, MIN_THROTTLE, MAX_THROTTLE);
  m1 = constrain(m1, MIN_THROTTLE, MAX_THROTTLE);
  m2 = constrain(m2, MIN_THROTTLE, MAX_THROTTLE);
  m3 = constrain(m3, MIN_THROTTLE, MAX_THROTTLE);
  #ifdef DEBUG_MOTORS_SPEED
  Serial.print("  m0 :  ");
  Serial.print(m0);
  Serial.print("  m1 :  ");
  Serial.print(m1);
  Serial.print("  m2 :  ");
  Serial.print(m2);
  Serial.print("  m3 :  ");
  Serial.print(m3);
  Serial.print("\n");
  #endif
  MOTOR_0.writeMicroseconds(m0);
  MOTOR_1.writeMicroseconds(m1);
  MOTOR_2.writeMicroseconds(m2);
  MOTOR_3.writeMicroseconds(m3);
}