/*
* Digital Sand Clock.ino
* A demo for ChaiHuo ZaoWuBa Demo
* 
* Copyright (c) 2015 Seeed Technology Inc.
* Auther     : Lmabor.Fang
* Create Time: May 2015
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

#define LIGHT_UP_TIME 15000//unit ms - 15 seconds delay

int led = OUT_PIN1;  //led control output pin
int pir = IN_PIN1;   //PIR_Sensor input pin

void setup() {
  WTD.watchdogSetup();
  WTD.doggieTickle();
  
  pinMode (LED1,OUTPUT);
  for(int i=0;i<2;i++)
  {
    analogWrite(LED1,5);
    delay(500);
    analogWrite(LED1,0);     
    delay(500);  
    WTD.doggieTickle();
  }
  
  pinMode(led, OUTPUT);
  pinMode(pir, INPUT);
  analogWrite(led,255);
  delay(500);
}

void loop() {
  if(digitalRead(pir))
  {    
    //turn on the light
    for(int i=255; i >= 0; i--)
    {
      long y = cal_circle_y(i, 255);   
      analogWrite(led,y);
      
      WTD.doggieTickle();
      delay(10);      
    }   
    
    unsigned long begin = millis();
    
    while(LIGHT_UP_TIME > millis() - begin)
    {
      if(digitalRead(pir))
      {
        //begin = millis();
      }
      
      WTD.doggieTickle();  
      delay(10);
    }
    
    //Turn off the light
    for(int i = 0; i<=255; i++)
    {
      long y = cal_circle_y(i, 255);    
      analogWrite(led,y);
      
      WTD.doggieTickle();
      delay(10);      
    }
    
    delay(1000);
  } 
  
  delay(20);      
  WTD.doggieTickle();
}

//r * r = (x-r)*(x-r) + y * y 
long cal_circle_y(long x, long r)
{
    return sqrt((r * r) - (x - r) * (x - r));
}
