/*
* Groot.ino
* A demo for ChaiHuo ZaoWuBa Demo T14006
*
* Copyright (c) 2015 Seeed Technology Inc.
* Auther     : Jacob.Yan
* Create Time: Jan 07, 2015
* Change Log :
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

/**
 * WatchDog class
 */
#include <avr/wdt.h>

#define VERSION "joint v3.0"
#define NAME    "loudness servo"
#define SKU     "110060106"

class WatchDog
{
  public:
    //initial watchdog timeout
    WatchDog(long timeout = 2000){
      _timeout = timeout;
    }

    //method
    void watchdogSetup(void){
      cli();
      wdt_reset();
      MCUSR &= ~(1<<WDRF);
      WDTCSR = (1<<WDCE) | (1<<WDE);
      WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);
      sei();
    }

    void doggieTickle(void){
      ResetTime = millis();
    }

    //reset
    void(* resetFunc) (void) = 0;

    //parameters
    unsigned long ResetTime;
    volatile bool  Flg_Power;
    long _timeout;

};

WatchDog WTD;

ISR(WDT_vect)
{
  if(millis() - WTD.ResetTime > WTD._timeout)
  {
    WTD.doggieTickle();
    WTD.resetFunc();
  }
}
/******************* End of WatchDog ********************/

#include <Wire.h>
#include <Servo.h>

#define  DEBUG   0

//hardware IO definition
#define BUTTON         2
#define LIGHT_SENSOR   A0
#define PWR_HOLD       A1
#define KEY            2
#define LED1           9
#define LED2           13
#define OUT_PIN1       5   //normal output pin
#define OUT_PIN2       6
#define IN_PIN1        A5  //normal input pin
#define IN_PIN2        A4

#define original_pos 90
// Servo position begin value
#define pos_begin  80
// Servo position end value
#define pos_end    100
// Create servo object to control a servo
Servo myservo;

// Sound sensor pin input define
const int pin_sound = IN_PIN1;

//sound analog value
int quiet_value = 0;
int val_sound = 0;

//value off the thershold,
int thershold_off = 50;
//sound threshold
int Threshold[5] = {40 + thershold_off,
                    60 + thershold_off,
                    80 + thershold_off,
                    100 + thershold_off,
                    120 + thershold_off};


const long interval = 50;
unsigned long previousMillis = 0;
unsigned long currentMillis  = 0;

int midNum(int *a, int *b, int *c);
int average_filter(int analog_pin, int num);
int mid_filter(int analog_pin);
void delay_feed( int val);
void servoRun(int analog);

void servoRun(int analog)
{
  int dec = 0;
  if(analog < 200)
  {
    dec = map(analog, thershold_off, 200, 2, 30);
  }
  else
  {
    dec = 30;
  }
  myservo.write(original_pos - dec);
  delay(200);
  myservo.write(original_pos + dec);
  delay(200);
}
// Delay with feed dog
void delay_feed( int val)
{
  delay(val);
  WTD.doggieTickle();
}

int mid_filter(int analog_pin)
{
  int a = analogRead(analog_pin);
  delayMicroseconds(10);
  int b = analogRead(analog_pin);
  delayMicroseconds(10);
  int c = analogRead(analog_pin);
  delayMicroseconds(10);

  return midNum(&a, &b, &c);
}

int average_filter(int analog_pin, int num)
{
  long temp = 0;
  for(int i=0;i<num;i++)
  {
    //temp += analogRead(analog_pin);
    temp += mid_filter(analog_pin);
  }
  return temp/num;
}

int midNum(int *a, int *b, int *c)
{
  int tmp = 0;
  if(*a > *b){
    tmp = *a;
    *a = *b;
    *b = tmp;
  }
  if(*b > *c){
    tmp = *b;
    *b = *c;
    *c = tmp;
  }
  if(*b > *c){
    tmp = *b;
    *b = *c;
    *c = tmp;
  }
  return *b;
}

void setup()
{
  Serial.begin(9600);
  Serial.print("Name: ");
  Serial.println(NAME);
  Serial.print("SKU: ");
  Serial.println(SKU);
  Serial.print("Version: ");
  Serial.println(VERSION);
  delay(100);

  //initial watchdog
  WTD.watchdogSetup();
  WTD.doggieTickle();

  //get quiet sound value
  delay(500);
  long tmp = 0;
  for(int i = 0;i<1000;i++)
  {
    tmp += analogRead(pin_sound);
  }
  quiet_value = tmp / 1000;

  pinMode (LED1,OUTPUT);
  for(int i=0;i<2;i++)
  {
    digitalWrite(LED1,HIGH);
    delay(500);
    digitalWrite(LED1,LOW);
    delay(500);
    WTD.doggieTickle();
  }

  #if DEBUG
  Serial.begin(9600);
  Serial.println("start");
  #endif

  myservo.attach(5);
  previousMillis = millis();
  myservo.write(90);

}

void loop()
{
  WTD.doggieTickle();
  val_sound = average_filter(pin_sound, 50);
  int threshold = val_sound - quiet_value;
  if(threshold > thershold_off)
  {
    Serial.println(threshold);
    servoRun(threshold);
  }
  WTD.doggieTickle();
  delay(200);
}
