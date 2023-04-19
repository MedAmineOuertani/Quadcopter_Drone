#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

float values_to_send[4];


RF24 radio(4, 5, 18, 19, 23);
const byte add[6] = "00001";

float map_with_step(float value, float low, float high, float tolow, float tohigh, float step) {
  // Calculate the range of the input values and output values
  float from_range = high - low;
  float to_range = tohigh - tolow;

  // Calculate the scaled value in the input range
  float scaled_value = (value - low) / (from_range / step);

  // Map the scaled value to the output range
  float mapped_value = tolow + (scaled_value * (to_range / step));

  return mapped_value;
}

void setup(void) {
  Serial.begin(9600);
   radio.begin();
  radio.setChannel(2);
  radio.setPayloadSize(16);
  //radio.setRetries(15, 15);
  //radio.setAutoAck(true);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(add);
   radio.setPALevel(RF24_PA_MIN);
  radio.stopListening(); 

  lcd.init(); //initialize the lcd
lcd.backlight();


}

void loop(void) {
  int throttle = analogRead(33);
  // I'm using three potentiometers for manipulating kp, ki and kd values.
  // change pins number when needed!!
  int kp = analogRead(34);
  int ki = analogRead(14);
  int kd = analogRead(27);

  values_to_send[0] = map(throttle, 0, 4095, 1500, 1000);

  // you can change the range and the step of each PID gains value.
  // they are not the same for everyone!!
  values_to_send[1] = map_with_step(kp, 0, 4095, 0.0, 1.5,0.02);
  values_to_send[2] = map_with_step(ki, 0, 4095, 0.0, 1.5, 0.01);
  values_to_send[3] = map_with_step(kd, 0, 4095, 0, 0.2, 0.01);

  Serial.print(" throttle : ");
  Serial.print(int16_t(values_to_send[0]));
  
  Serial.print(" kp : ");
  Serial.print(values_to_send[1]);
  Serial.print(" ki : ");
  Serial.print(values_to_send[2]);
  Serial.print(" kd : ");
  Serial.print(values_to_send[3]);
  Serial.print("\n");

// you can delete this code if you dont have an LCD
  lcd.clear();                 // clear display
  lcd.setCursor(0, 0);
 lcd.print("kp");
  lcd.print(values_to_send[1]);
  lcd.print("ki");
  lcd.print(values_to_send[2]);
  
lcd.setCursor(0, 1);
lcd.print("kd");
  lcd.print(values_to_send[3]);
  //lcd.print("throttle: "); // move cursor to   (0, 0)
  //lcd.print(int16_t(values_to_send[0]));
  

  
 
  radio.write(&values_to_send, sizeof(float)*4);
  
}