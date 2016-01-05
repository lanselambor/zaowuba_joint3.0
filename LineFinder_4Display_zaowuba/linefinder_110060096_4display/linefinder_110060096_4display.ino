/*
* Coin counter.ino
* A demo for ChaiHuo ZaoWuBa Demo T14015
* 
* Copyright (c) 2015 Seeed Technology Inc.
* Auther     : Jacob.Yan
* Create Time: Jan 07, 2015
* Change Log : Lambor modified and update at May 2015
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
/******************* End of Class WatchDog ********************/


/**
 * Class TM1637 
 */
 
#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44
#define STARTADDR  0xc0 
#define POINT_ON   1
#define POINT_OFF  0
#define  BRIGHT_DARKEST 0
#define  BRIGHT_TYPICAL 2
#define  BRIGHTEST      7

static int8_t TubeTab[] = {0x3f,0x06,0x5b,0x4f,
                           0x66,0x6d,0x7d,0x07,
                           0x7f,0x6f,0x77,0x7c,
                           0x39,0x5e,0x79,0x71};//0~9,A,b,C,d,E,F  
                           
class TM1637
{
  public:
    uint8_t Cmd_SetData;
    uint8_t Cmd_SetAddr;
    uint8_t Cmd_DispCtrl;
    boolean _PointFlag;     //_PointFlag=1:the clock point on
    
    TM1637(uint8_t Clk, uint8_t Data)
    {
      Clkpin = Clk;
      Datapin = Data;
      pinMode(Clkpin,OUTPUT);
      pinMode(Datapin,OUTPUT);
    }
    
    void init(void)        //To clear the display
    {
      clearDisplay();
    }
    bool writeByte(int8_t wr_data) //write 8bit data to tm1637
    {
      uint8_t i,count1; 
      uint8_t ack = 0;  
      for(i=0;i<8;i++)        //sent 8bit data
      {
        digitalWrite(Clkpin,LOW);      
        if(wr_data & 0x01)digitalWrite(Datapin,HIGH);//LSB first
        else digitalWrite(Datapin,LOW);
        wr_data >>= 1;      
        digitalWrite(Clkpin,HIGH);
          
      }  
      digitalWrite(Clkpin,LOW); //wait for the ACK
      digitalWrite(Datapin,HIGH);
      digitalWrite(Clkpin,HIGH);     
      pinMode(Datapin,INPUT);

    #if 0  
      while(digitalRead(Datapin))    
      { 
        count1 +=1;
        if(count1 == 200)//
        {
         pinMode(Datapin,OUTPUT);
         digitalWrite(Datapin,LOW);
         count1 =0;
        }
        pinMode(Datapin,INPUT);
      }
      pinMode(Datapin,OUTPUT);
    #else
      bitDelay();
      ack = digitalRead(Datapin);
      if (ack == 0) 
      {
         pinMode(Datapin,OUTPUT);
         digitalWrite(Datapin,LOW);
      }
      bitDelay();
      pinMode(Datapin,OUTPUT);
      bitDelay();
    #endif  
      return ack;
    }
    
    void start(void) //send start bits
    {
      digitalWrite(Clkpin,HIGH);//send start signal to TM1637
      digitalWrite(Datapin,HIGH); 
      digitalWrite(Datapin,LOW); 
      digitalWrite(Clkpin,LOW);
    }
    
    void stop(void) //send stop bits
    {
      digitalWrite(Clkpin,LOW);
      digitalWrite(Datapin,LOW);
      digitalWrite(Clkpin,HIGH);
      digitalWrite(Datapin,HIGH);
    }
    
    void display(int8_t DispData[])
    {
      int8_t SegData[4];
      uint8_t i;
      for(i = 0;i < 4;i ++)
      {
        SegData[i] = DispData[i];
      }
      coding(SegData);
      start();          //start signal sent to TM1637 from MCU
      writeByte(ADDR_AUTO);//
      stop();           //
      start();          //
      writeByte(Cmd_SetAddr);//
      for(i=0;i < 4;i ++)
      {
        writeByte(SegData[i]);        //
      }
      stop();           //
      start();          //
      writeByte(Cmd_DispCtrl);//
      stop();           //
    }
    
    void display(uint8_t BitAddr,int8_t DispData)
    {
      int8_t SegData;
      SegData = coding(DispData);
      start();          //start signal sent to TM1637 from MCU
      writeByte(ADDR_FIXED);//
      stop();           //
      start();          //
      writeByte(BitAddr|0xc0);//
      writeByte(SegData);//
      stop();            //
      start();          //
      writeByte(Cmd_DispCtrl);//
      stop();           //
    }
    
    void clearDisplay(void)
    {
      display(0x00,0x7f);
      display(0x01,0x7f);
      display(0x02,0x7f);
      display(0x03,0x7f); 
    }

    //To take effect the next time it displays.
    void set(uint8_t brightness = BRIGHT_TYPICAL,uint8_t SetData = 0x40,uint8_t SetAddr = 0xc0) 
    {
      Cmd_SetData = SetData;
      Cmd_SetAddr = SetAddr;
      Cmd_DispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
    }
    
    //Whether to light the clock point ":".
    //To take effect the next time it displays.
    void point(boolean PointFlag)//whether to light the clock point ":".To take effect the next time it displays.
    {
      _PointFlag = PointFlag;
    }
    
    void coding(int8_t DispData[])
    {
      uint8_t PointData;
      if(_PointFlag == POINT_ON)PointData = 0x80;
      else PointData = 0; 
      for(uint8_t i = 0;i < 4;i ++)
      {
        if(DispData[i] == 0x7f)DispData[i] = 0x00;
        else DispData[i] = TubeTab[DispData[i]] + PointData;
      }
    }
    
    int8_t coding(int8_t DispData)
    {
      uint8_t PointData;
      
      if(_PointFlag == POINT_ON)PointData = 0x80;
      else PointData = 0; 
      if(DispData == 0x7f) DispData = 0x00 + PointData;//The bit digital tube off
      else DispData = TubeTab[DispData] + PointData;
      return DispData;
    }
    void bitDelay(void)
    {
      delayMicroseconds(50);
    }
    
  private:
    uint8_t Clkpin;
    uint8_t Datapin;
};
/************************ End of TM1637 **********************/


#include <Wire.h>
#include <EEPROM.h>


//DeBug  switch 
#define  DeBug   0

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

#define CLK OUT_PIN1      
#define DIO OUT_PIN2

TM1637 tm1637(CLK,DIO);
int16_t ListDisp[4];

const int linefinder =A5;
volatile  uint16_t num_money = 0;
int begin = 0;
int end = 0;

void display_seg(uint16_t num_money)
{
  ListDisp[0] = num_money%10;
  ListDisp[1] = (num_money/10)%10;  
  ListDisp[2] = (num_money/100)%10;
  ListDisp[3] = (num_money/1000)%10;  

  if (ListDisp[3]!=0)
  {
    tm1637.display(0,ListDisp[3]);
    tm1637.display(1,ListDisp[2]); 
    tm1637.display(2,ListDisp[1]);
    tm1637.display(3,ListDisp[0]);      
  }
  
  if ((ListDisp[3]==0)&&(ListDisp[2]!=0))
  {
    tm1637.display(1,ListDisp[2]); 
    tm1637.display(2,ListDisp[1]);
    tm1637.display(3,ListDisp[0]);  
  }

  if ((ListDisp[3]==0)&&(ListDisp[2]==0)&&(ListDisp[1]!=0))
  {
    tm1637.display(2,ListDisp[1]);
    tm1637.display(3,ListDisp[0]);    
  }
        
  if ((ListDisp[3]==0)&&(ListDisp[2]==0)&&(ListDisp[1]==0)&&(ListDisp[0]!=0))
  {
    tm1637.display(3,ListDisp[0]);    
  }
}

void setup()
{         
  WTD.watchdogSetup();
  WTD.doggieTickle();
  
  pinMode (linefinder,INPUT);
  tm1637.init();
  tm1637.set(BRIGHT_TYPICAL);  
  tm1637.clearDisplay();
  tm1637.display(3,0);
  
#if 0  // Don't erase eeprom when restarting
  if (EEPROM.read(3)!=0)
  {
    for(int i=0;i<10;i++)EEPROM.write(i,0);
    
  }
#endif  
  
  pinMode (LED1,OUTPUT);
  for(int i=0;i<2;i++)
  {
        analogWrite(LED1,5);
        delay(500);
        analogWrite(LED1,0);     
        delay(500);  
        WTD.doggieTickle();
  }
  //while(1);   //debug  watchdog   
#if DeBug  
    Serial.begin(9600);
  Serial.println("start");
#endif    
}

void loop()
{      
  WTD.doggieTickle();
         
  // Keep reading from eeprom ,then display data on 4-digital segment
  num_money = EEPROM.read(1);
  num_money = num_money << 8;                     // Read form eeprom 
  num_money = num_money + EEPROM.read(0);  
  
  display_seg(num_money);   

  // If detect proximity
  if (digitalRead(linefinder)==LOW)                  // When detect proximity ,output LOW  
  {
    begin = millis();
    num_money ++;
    display_seg(num_money);                    // Display      

#if DeBug      
    Serial.println(num_money);
#endif   
    for(int i=0; i<2;i++)
    {
            EEPROM.write(i,num_money & 0xff);        // Write into eeprom  
            num_money = num_money >> 8;  
    }

    while (digitalRead(linefinder) ==LOW)
    {
      
      WTD.doggieTickle();
      end = millis();
      if (end - begin >3000)
      {
        num_money = 0;
        for(int i=0; i<2;i++)
        {
          EEPROM.write(i,0);
        }          
        tm1637.clearDisplay();
        tm1637.display(3,0);
        //Serial.println("zero");
      }
    }
  }
}


