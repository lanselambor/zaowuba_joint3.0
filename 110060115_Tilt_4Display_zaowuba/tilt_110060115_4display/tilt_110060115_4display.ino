/*
* Digital Sand Clock.ino
* A demo for ChaiHuo ZaoWuBa Demo
*
* Copyright (c) 2015 Seeed Technology Inc.
* Auther     : Wayen.Weng
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
 *  Class TimeIne
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#define RESOLUTION 65536    // Timer1 is 16 bit

class TimerOne
{
  public:

    // properties
    unsigned int pwmPeriod;
    unsigned char clockSelectBits;
	  char oldSREG;					// To hold Status Register while ints disabled

    // methods
    void initialize(long microseconds=1000000)
    {
      TCCR1A = 0;                 // clear control register A
      TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
      setPeriod(microseconds);
    }

    void start()
    {
      unsigned int tcnt1;

      TIMSK1 &= ~_BV(TOIE1);        // AR added
      GTCCR |= _BV(PSRSYNC);   		// AR added - reset prescaler (NB: shared with all 16 bit timers);

      oldSREG = SREG;				// AR - save status register
      cli();						// AR - Disable interrupts
      TCNT1 = 0;
      SREG = oldSREG;          		// AR - Restore status register
      resume();
      do {	// Nothing -- wait until timer moved on from zero - otherwise get a phantom interrupt
      oldSREG = SREG;
      cli();
      tcnt1 = TCNT1;
      SREG = oldSREG;
      } while (tcnt1==0);

    //  TIFR1 = 0xff;              		// AR - Clear interrupt flags
    //  TIMSK1 = _BV(TOIE1);              // sets the timer overflow interrupt enable bit
    }

    void stop()
    {
      TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));          // clears all clock selects bits
    }

    void restart()
    {
      start();
    }

	  void resume()
    {
      TCCR1B |= clockSelectBits;
    }

	  unsigned long read()        //returns the value of the timer in microseconds
    {                           //rember! phase and freq correct mode counts up to then down again
      unsigned long tmp;				// AR amended to hold more than 65536 (could be nearly double this)
      unsigned int tcnt1;				// AR added

      oldSREG= SREG;
        cli();
        tmp=TCNT1;
      SREG = oldSREG;

      char scale=0;
      switch (clockSelectBits)
      {
      case 1:// no prescalse
        scale=0;
        break;
      case 2:// x8 prescale
        scale=3;
        break;
      case 3:// x64
        scale=6;
        break;
      case 4:// x256
        scale=8;
        break;
      case 5:// x1024
        scale=10;
        break;
      }

      do {	// Nothing -- max delay here is ~1023 cycles.  AR modified
        oldSREG = SREG;
        cli();
        tcnt1 = TCNT1;
        SREG = oldSREG;
      } while (tcnt1==tmp); //if the timer has not ticked yet

      //if we are counting down add the top value to how far we have counted down
      tmp = (  (tcnt1>tmp) ? (tmp) : (long)(ICR1-tcnt1)+(long)ICR1  );		// AR amended to add casts and reuse previous TCNT1
      return ((tmp*1000L)/(F_CPU /1000L))<<scale;
    }

    void pwm(char pin, int duty, long microseconds=-1)
    {
      if(microseconds > 0) setPeriod(microseconds);
      if(pin == 1 || pin == 9) {
        DDRB |= _BV(PORTB1);                                   // sets data direction register for pwm output pin
        TCCR1A |= _BV(COM1A1);                                 // activates the output pin
      }
      else if(pin == 2 || pin == 10) {
        DDRB |= _BV(PORTB2);
        TCCR1A |= _BV(COM1B1);
      }
      setPwmDuty(pin, duty);
      resume();			// Lex - make sure the clock is running.  We don't want to restart the count, in case we are starting the second WGM
                    // and the first one is in the middle of a cycle
    }

    void disablePwm(char pin)
    {
      if(pin == 1 || pin == 9)       TCCR1A &= ~_BV(COM1A1);   // clear the bit that enables pwm on PB1
      else if(pin == 2 || pin == 10) TCCR1A &= ~_BV(COM1B1);   // clear the bit that enables pwm on PB2
    }

    void attachInterrupt(void (*isr)(), long microseconds=-1)
    {
      if(microseconds > 0) setPeriod(microseconds);
      isrCallback = isr;                                       // register the user's callback with the real ISR
      TIMSK1 = _BV(TOIE1);                                     // sets the timer overflow interrupt enable bit
      // might be running with interrupts disabled (eg inside an ISR), so don't touch the global state
      //  sei();
      resume();
    }

    void detachInterrupt()
    {
      TIMSK1 &= ~_BV(TOIE1);  // clears the timer overflow interrupt enable bit
															// timer continues to count without calling the isr
    }

    void setPeriod(long microseconds)
    {
      long cycles = (F_CPU / 2000000) * microseconds;                                // the counter runs backwards after TOP, interrupt is at BOTTOM so divide microseconds by 2
      if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
      else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
      else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
      else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
      else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
      else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum

      oldSREG = SREG;
      cli();							// Disable interrupts for 16 bit register access
      ICR1 = pwmPeriod = cycles;                                          // ICR1 is TOP in p & f correct pwm mode
      SREG = oldSREG;

      TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
      TCCR1B |= clockSelectBits;
    }

    void setPwmDuty(char pin, int duty)
    {
      unsigned long dutyCycle = pwmPeriod;

      dutyCycle *= duty;
      dutyCycle >>= 10;

      oldSREG = SREG;
      cli();
      if(pin == 1 || pin == 9)       OCR1A = dutyCycle;
      else if(pin == 2 || pin == 10) OCR1B = dutyCycle;
      SREG = oldSREG;
    }

    void (*isrCallback)();
};

TimerOne Timer1;              // preinstatiate

ISR(TIMER1_OVF_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  Timer1.isrCallback();
}

/*************** End of TimeOne ***************************/

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

#include <inttypes.h>
//************definitions for TM1637*********************
#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44

#define STARTADDR  0xc0
/**** definitions for the clock point of the digit tube *******/
#define POINT_ON   1
#define POINT_OFF  0
/**************definitions for brightness***********************/
#define  BRIGHT_DARKEST 0
#define  BRIGHT_TYPICAL 2
#define  BRIGHTEST      7
/**************definitions for display flag***********************/
#define  DISPLAY_FLAG_F  0
#define  DISPLAY_FLAG_B  1

static int8_t TubeTabF[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71};//0~9,A,b,C,d,E,F
static int8_t TubeTabB[] = {0x3f,0x30,0x5b,0x79,0x74,0x6d,0x6f,0x38,0x7f,0x7d,0x7e,0x67,0x0f,0x73,0x4f,0x4e};//0~9,A,b,C,d,E,F

class MyTM1637
{
  public:
    uint8_t Cmd_SetData;
    uint8_t Cmd_SetAddr;
    uint8_t Cmd_DispCtrl;
    boolean _PointFlag;     //_PointFlag=1:the clock point on

    MyTM1637(uint8_t Clk, uint8_t Data)
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

    int writeByte(int8_t wr_data) //write 8bit data to tm1637
    {
      uint8_t i,count1;
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
    #endif

      bitDelay();
      uint8_t ack = digitalRead(Datapin);
      if (ack == 0)
      {
         pinMode(Datapin,OUTPUT);
         digitalWrite(Datapin,LOW);
      }
      bitDelay();
      pinMode(Datapin,OUTPUT);
      bitDelay();

      return ack;
    }

    void start(void) //send start bits
    {
      digitalWrite(Clkpin,HIGH);//send start signal to TM1637
      digitalWrite(Datapin,HIGH);
      digitalWrite(Datapin,LOW);
      digitalWrite(Clkpin,LOW);
    }

    void stop(void)  //send stop bits
    {
      digitalWrite(Clkpin,LOW);
      digitalWrite(Datapin,LOW);
      digitalWrite(Clkpin,HIGH);
      digitalWrite(Datapin,HIGH);
    }

    void display(int8_t DispData[],uint8_t DispFlag)
    {
      int8_t SegData[4];
      uint8_t i;
      if(DispFlag == DISPLAY_FLAG_F)
      {
          for(i = 0;i < 4;i ++)
          {
            SegData[i] = DispData[i];
          }
      }
      if(DispFlag == DISPLAY_FLAG_B)
      {
          for(i = 0;i < 4;i ++)
          {
            SegData[3 - i] = DispData[i];
          }
      }
      coding(SegData,DispFlag);
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

    void display(uint8_t BitAddr,int8_t DispData,uint8_t DispFlag)
    {
      int8_t SegData;
      SegData = coding(DispData,DispFlag);
      start();          //start signal sent to TM1637 from MCU
      writeByte(ADDR_FIXED);//
      stop();           //
      start();          //
      if(DispFlag == DISPLAY_FLAG_F)writeByte(BitAddr|0xc0);//
      if(DispFlag == DISPLAY_FLAG_B)writeByte((3 - BitAddr)|0xc0);//
      writeByte(SegData);//
      stop();            //
      start();          //
      writeByte(Cmd_DispCtrl);//
      stop();           //
    }

    void clearDisplay(void)
    {
      display(0x00,0x7f,0x00);
      display(0x01,0x7f,0x00);
      display(0x02,0x7f,0x00);
      display(0x03,0x7f,0x00);
    }

    void set(uint8_t brightness = BRIGHT_TYPICAL,uint8_t SetData = 0x40,uint8_t SetAddr = 0xc0)  //To take effect the next time it displays.
    {
      Cmd_SetData = SetData;
      Cmd_SetAddr = SetAddr;
      Cmd_DispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
    }

    void point(boolean PointFlag) //whether to light the clock point ":".To take effect the next time it displays.
    {
      _PointFlag = PointFlag;
    }

    void coding(int8_t DispData[],uint8_t DispFlag)
    {
      uint8_t PointData;
      if(DispFlag == DISPLAY_FLAG_F)
      {
          if(_PointFlag == POINT_ON)PointData = 0x80;
          else PointData = 0;
          for(uint8_t i = 0;i < 4;i ++)
          {
            if(DispData[i] == 0x7f)DispData[i] = 0x00;
            else DispData[i] = TubeTabF[DispData[i]] + PointData;
          }
      }
      if(DispFlag == DISPLAY_FLAG_B)
      {
          if(_PointFlag == POINT_ON)PointData = 0x80;
          else PointData = 0;
          for(uint8_t i = 0;i < 4;i ++)
          {
            if(DispData[i] == 0x7f)DispData[i] = 0x00;
            else DispData[i] = TubeTabB[DispData[i]] + PointData;
          }
      }
    }

    int8_t coding(int8_t DispData,uint8_t DispFlag)
    {
      uint8_t PointData;
      if(DispFlag == DISPLAY_FLAG_F)
      {
          if(_PointFlag == POINT_ON)PointData = 0x80;
          else PointData = 0;
          if(DispData == 0x7f) DispData = 0x00 + PointData;//The bit digital tube off
          else DispData = TubeTabF[DispData] + PointData;
      }
      if(DispFlag == DISPLAY_FLAG_B)
      {
          if(_PointFlag == POINT_ON)PointData = 0x80;
          else PointData = 0;
          if(DispData == 0x7f) DispData = 0x00 + PointData;//The bit digital tube off
          else DispData = TubeTabB[DispData] + PointData;
      }
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
/****************** End of My TM1637 ******************/

#include <EEPROM.h>
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

#define TILT_PIN      IN_PIN1
#define CLK           OUT_PIN1
#define DIO           OUT_PIN2
#define ON            1
#define OFF           0

MyTM1637 tm1637(CLK,DIO);

unsigned char MinuteMax = 25;
int8_t TimeDisp[] = {0,0,0,0};
unsigned char ClockPoint = 1;
unsigned char Update;
unsigned char halfsecond = 0;
unsigned char second = 0;
unsigned char minute = 0;
unsigned char displayflag = 0;
unsigned char tilt_value;

void TimingISR()
{
  if(displayflag == 1)
  {
    halfsecond ++;
    Update = ON;
    if(halfsecond == 2)
    {
      if(second == 0)
      {
        if(minute == 0)
        {
            //minute = 25;
        }
        minute --;
        second = 60;
      }
      second --;
      halfsecond = 0;
    }
    ClockPoint = (~ClockPoint) & 0x01;
  }

  WTD.doggieTickle();
}

void TimeUpdate(void)
{
  if(ClockPoint)tm1637.point(POINT_ON);
  else tm1637.point(POINT_OFF);
  TimeDisp[0] = minute / 10;
  TimeDisp[1] = minute % 10;
  TimeDisp[2] = second / 10;
  TimeDisp[3] = second % 10;
  Update = OFF;
}

void setup()
{
  WTD.watchdogSetup();
  WTD.doggieTickle();

  pinMode (LED1,OUTPUT);
  for(int i=0;i<2;i++)
  {
    digitalWrite(LED1, HIGH);
    delay(500);
    digitalWrite(LED1, LOW); 
    delay(500);
    WTD.doggieTickle();
  }

  Serial.begin(9600);
  Serial.print("Digital sand clock Test.\r\n");

  tm1637.init();
  tm1637.set(BRIGHT_TYPICAL);
  tm1637.clearDisplay();

  pinMode(TILT_PIN,INPUT_PULLUP);

  Timer1.initialize(500000);//timing for 500ms
  Timer1.attachInterrupt(TimingISR);//declare the interrupt serve routine:TimingISR

  EEPROM.write(0,MinuteMax); //Setting minuteMax, the initial value of count time  max.

  MinuteMax = EEPROM.read(0);
  Serial.print("Read time max is: ");
  Serial.print(MinuteMax);
  Serial.print(" minutes.\r\n");

  minute = MinuteMax;
}

void loop()
{
  if(tilt_value != digitalRead(TILT_PIN))
  {
    tilt_value = digitalRead(TILT_PIN);
    minute = MinuteMax;
    second = 0;
    displayflag = 1;
  }

  if(Update == ON)
  {
    TimeUpdate();
    if(tilt_value == 1) tm1637.display(TimeDisp,DISPLAY_FLAG_F);
    if(tilt_value == 0) tm1637.display(TimeDisp,DISPLAY_FLAG_B);
  }

  if((minute == 0) && (second == 0))
  {
    displayflag = 0;
    ClockPoint = 1;
    minute = 0;
    second = 0;
    TimeUpdate();
    tm1637.display(TimeDisp,DISPLAY_FLAG_F);
  }

#if 0
  /** Here you type in a number as minutes,
    * for the 4 digital display can only show the largest time 99:59,
    * you should type in numbers between 0000 ~ 0100.
    */
  String inString = "";
  unsigned int timeValue = 0;
  unsigned char bitCount = 0;
  unsigned char dataTemp1[4] = {0};
  while(Serial.available() > 0)
  {
    unsigned char inChar = Serial.read();
    inString += (char)inChar;
    dataTemp1[bitCount] = inChar - '0';
    bitCount += 1;
    delay(10);
  }
  if(inString != "")
  {
    //Serial.print("Into time max setup.\r\n");
    //delay(100);

    if(bitCount > 4)
    {
        Serial.print("Time input error.\r\n");
    }
    else
    {
      for(char i=0;i<bitCount;i++)
      {
        unsigned int dataTemp2 = 1;
        for(char j=0;j<(bitCount-i-1);j++)dataTemp2 *= 10;
        timeValue += (dataTemp1[i] * dataTemp2);
      }
      if(timeValue != MinuteMax)
      {
        MinuteMax = timeValue;
        minute = MinuteMax;
        second = 0;
        EEPROM.write(0,MinuteMax);
        Serial.print("Write the time max: ");
        Serial.print(MinuteMax);
        Serial.print(" minutes.\r\n");
      }
    }
  }
#endif

  WTD.doggieTickle();
}
