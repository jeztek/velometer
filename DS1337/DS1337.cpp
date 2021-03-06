/*
 Yet-Another DS1337 RTC Library
 By Tim Gipson (drmn4ea at google's mail)
 
 Based loosely on mattt and xSmurf's RTC library at (http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1191209057/0)
 epoch_seconds_to_date() function based on this Dallas/Maxim application note: http://www.maxim-ic.com/app-notes/index.mvp/id/517
 
 This library is written for use with the Mosquino (http://code.google.com/p/mosquino/),
 but should be adaptable to other boards with no/minimal changes 
 (mainly just changing the RTC_INT_NUMBER in DS1337.h if different, or modifying the alarm 
 functions to work without interrupts if needed).
 
*/

extern "C" {

	#include <avr/power.h>
	#include <avr/sleep.h>

	// Dummy "interrupt handler" for sleep to wake up to on alarm interrupt
	void _dummy_int_handler(void)
	{

	}
	
}

//#include <Arduino.h>
#include "DS1337.h"
#include <Wire.h>

// NOTE: To keep the math from getting even more lengthy/annoying than it already is, the following constraints are imposed:
//   1) All times are in 24-hour format (military time)
//   2) DayOfWeek field is not used internally or checked for validity. Alarm functions may optionally set alarms repeating on DayOfWeek, but this feature has not been tested yet.
//   3) This library's buffer stores all times in raw BCD format, just as it is sent from the RTC.
//      It is not converted to/from 'real' (binary) values until needed via get...() and set...() functions.
//      In other words, don't go hacking around and reading from the rtc_bcd[] buffer directly, unless you want the raw BCD results.


// Cumulative number of days elapsed at the start of each month, assuming a normal (non-leap) year.
const unsigned int monthdays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

DS1337::DS1337()
{
	Wire.begin();
	pinMode(RTC_INT_PIN, INPUT);
	digitalWrite(RTC_INT_PIN, HIGH);	// enable software pullup resistor on RTC interrupt pin
}

// Aquire data from the RTC chip in BCD format
// refresh the buffer
void DS1337::readTime(void)
{
// use the Wire lib to connect to tho rtc
// reset the register pointer to zero
	Wire.beginTransmission(DS1337_CTRL_ID);
	I2C_WRITE((uint8_t)0x00); // Explicit cast is to hack around http://code.google.com/p/arduino/issues/detail?id=527
	Wire.endTransmission();

// request the 7 bytes of data    (secs, min, hr, dow, date. mth, yr)
	Wire.requestFrom(DS1337_CTRL_ID, 7);
	for(int i=0; i<7; i++)
	{
	// store data in raw bcd format
		if (Wire.available())
			rtc_bcd[i]=I2C_READ();
	}
}

// Read the current alarm value. Note that the repeat flags and DY/DT are removed from the result.
void DS1337::readAlarm(void)
{
        //alarm_repeat = 0;
        byte temp;
// use the Wire lib to connect to tho rtc
// point to start of Alarm1 registers
	Wire.beginTransmission(DS1337_CTRL_ID);
	I2C_WRITE((uint8_t)DS1337_ARLM1);
	Wire.endTransmission();

// request the *4* bytes of data (secs, min, hr, dow/date). Note the format is nearly identical, except for the choice of dayOfWeek vs. date,
// and that the topmost bit of each helps determine if/how the alarm repeats.
	Wire.requestFrom(DS1337_CTRL_ID, 4);
	for(int i=0; i<4; i++)
	{
                // store data in raw bcd format
		if (Wire.available())
		{
			temp = I2C_READ();
			rtc_bcd[i] = temp & B01111111;
		}
	}

	// 4th byte read may contain either a date or DayOfWeek, depending on the value of the DY/DT flag.
	// For laziness sake we read it into the DayOfWeek field regardless (rtc_bcd[3]). Correct as needed...
        if(rtc_bcd[3] & B01000000) // DY/DT set: DayOfWeek
        {
           rtc_bcd[3] &= B10111111; // clear DY/DT flag
           rtc_bcd[4] = 0; // alarm *date* undefined
        }
        else
        {
            rtc_bcd[4] = rtc_bcd[3];
            rtc_bcd[3] = 0; // alarm dayOfWeek undefined
        }
}

// update the data on the IC from the bcd formatted data in the buffer

void DS1337::writeTime(void)
{
	Wire.beginTransmission(DS1337_CTRL_ID);
	I2C_WRITE((uint8_t)0x00); // reset register pointer
	for(int i=0; i<7; i++)
	{
		I2C_WRITE(rtc_bcd[i]);
	}
	Wire.endTransmission();

	// clear the Oscillator Stop Flag
        setRegister(DS1337_STATUS, getRegister(DS1337_STATUS) & !DS1337_STATUS_OSF);
}

void DS1337::writeTime(unsigned long sse)
{
        epoch_seconds_to_date(sse);
        writeTime();
}

// FIXME: automatically set alarm interrupt after writing new alarm? Nah...

// Write the BCD alarm value in the buffer to the alarm registers.
// If an alarm repeat mode has been specified, poke those bytes into the buffer before sending.
void DS1337::writeAlarm(void)
{
	Wire.beginTransmission(DS1337_CTRL_ID);
	I2C_WRITE((uint8_t)DS1337_ARLM1); // set register pointer

        I2C_WRITE(rtc_bcd[DS1337_SEC] | ((alarm_repeat & B00000001 ) << 7)); // A1M1
        I2C_WRITE(rtc_bcd[DS1337_MIN] | ((alarm_repeat & B00000010 ) << 6)); // A1M2
        I2C_WRITE(rtc_bcd[DS1337_HR] | ((alarm_repeat & B00000100 ) << 5)); // A1M3

        // Check if we are using date or DayOfWeek and send the appropriate value
        if(alarm_repeat & B00001000) // DayOfWeek
        {
            // send DOW as 4th alarm reg byte
            I2C_WRITE(rtc_bcd[DS1337_DOW] | ((alarm_repeat & B00011000 ) << 3)); // A1M4 and DY/DT
        }
        else // date
        {
            // send date as 4th alarm reg byte
            I2C_WRITE(rtc_bcd[DS1337_DATE] | ((alarm_repeat & B00011000 ) << 3)); // A1M4 and DY/DT
        }

	Wire.endTransmission();
}


void DS1337::writeAlarm(unsigned long sse)
{
        epoch_seconds_to_date(sse);
        writeAlarm();
}

void DS1337::setAlarmRepeat(byte repeat)
{
        alarm_repeat = repeat;
}


unsigned char DS1337::getRegister(unsigned char registerNumber)
{
	Wire.beginTransmission(DS1337_CTRL_ID);
	I2C_WRITE(registerNumber);
	Wire.endTransmission();

	Wire.requestFrom(DS1337_CTRL_ID, 1);

	return I2C_READ();
}

void DS1337::setRegister(unsigned char registerNumber, unsigned char value)
{
	Wire.beginTransmission(DS1337_CTRL_ID);
	I2C_WRITE(registerNumber); // set register pointer

	I2C_WRITE(value);

	Wire.endTransmission();
}

unsigned char DS1337::time_is_set()
{
  // Return TRUE if Oscillator Stop Flag is clear (osc. not stopped since last time setting), FALSE otherwise
  byte asdf = ((getRegister(DS1337_STATUS) & DS1337_STATUS_OSF) == 0);
  return asdf;
}
unsigned char DS1337::alarm_is_set()
{
  // Return TRUE if the alarm interrupt flag is enabled.
  byte asdf = (getRegister(DS1337_SP) & DS1337_SP_A1IE);
  return asdf;
}

void DS1337::enable_interrupt()
{
   clear_interrupt();
   setRegister(DS1337_SP, getRegister(DS1337_SP) | DS1337_SP_INTCN | DS1337_SP_A1IE); // map alarm interrupt to INT1 and enable interrupt
}

void DS1337::disable_interrupt()
{
   setRegister(DS1337_SP, getRegister(DS1337_SP) & !DS1337_SP_A1IE);
}

void DS1337::clear_interrupt()
{
   setRegister(DS1337_STATUS, getRegister(DS1337_STATUS) & !DS1337_STATUS_A1F);
}

unsigned char DS1337::getSeconds()
{
    return bcd2bin(rtc_bcd[DS1337_SEC]);
}

unsigned char DS1337::getMinutes()
{
    return bcd2bin(rtc_bcd[DS1337_MIN]);
}
unsigned char DS1337::getHours()
{
    return bcd2bin(rtc_bcd[DS1337_HR]);
}
unsigned char DS1337::getDays()
{
    return bcd2bin(rtc_bcd[DS1337_DATE]);
}
unsigned char DS1337::getDayOfWeek()
{
    return bcd2bin(rtc_bcd[DS1337_DOW]);
}
unsigned char DS1337::getMonths()
{
    return bcd2bin(rtc_bcd[DS1337_MTH]);
}
unsigned int DS1337::getYears()
{
    return 2000 + bcd2bin(rtc_bcd[DS1337_YR]);
}


unsigned long DS1337::date_to_epoch_seconds(unsigned int year, byte month, byte day, byte hour, byte minute, byte second)
{

  //gracefully handle 2- and 4-digit year formats
  if (year > 1999)
  {
     year -= 2000;
  }


// Between year 2000 and 2100, a leap year occurs in every year divisible by 4.

//   sse_y = (((unsigned long)year)*365*24*60*60);
//   sse_ly = ((((unsigned long)year+3)>>2) + ((unsigned long)year%4==0 && (unsigned long)month>2))*24*60*60;
//   sse_d = ((unsigned long)monthdays[month-1] + (unsigned long)day-1) *24*60*60;
//   sse_h = ((unsigned long)hour*60*60);
//   sse_m = ((unsigned long)minute*60);
//   sse_s = (unsigned long)second;
//
//   sse = sse_y + sse_ly + sse_d + sse_h + sse_m + sse_s;



// NB: The multiplication-by-constants below is intentionally left expanded for readability; GCC is smart and will optimize them to single constants during compilation.


  //         Whole year seconds                      Cumulative total of seconds contributed by elapsed leap year days
  unsigned long sse = (((unsigned long)year)*365*24*60*60)   +   ((((unsigned long)year+3)>>2) + ((unsigned long)year%4==0 && (unsigned long)month>2))*24*60*60   +   \
         ((unsigned long)monthdays[month-1] + (unsigned long)day-1) *24*60*60   +   ((unsigned long)hour*60*60)   +   ((unsigned long)minute*60)   + (unsigned long)second;
         // Seconds in days since start of year                      hours                      minutes           sec
  sse += 946684800; // correct for difference between DS1337 epoch and UNIX epoch
  return sse;
}


unsigned long DS1337::date_to_epoch_seconds()
{
     unsigned long asdf = date_to_epoch_seconds(int(bcd2bin(rtc_bcd[DS1337_YR])), bcd2bin(rtc_bcd[DS1337_MTH]), bcd2bin(rtc_bcd[DS1337_DATE]), bcd2bin(rtc_bcd[DS1337_HR]), bcd2bin(rtc_bcd[DS1337_MIN]), bcd2bin(rtc_bcd[DS1337_SEC]));
     return asdf;
}



void DS1337::epoch_seconds_to_date(unsigned long seconds_left)
{
   // This routine taken from Dallas/Maxim application note 517
   // http://www.maxim-ic.com/app-notes/index.mvp/id/517
   // Arn't the fastest thing, but it produces correct results.

   // NOTE: The earliest date that can be represented by the DS1337 is 1/1/2000 (946684800 in Unix epoch seconds).
   // Passing an earlier Unix time stamp will fail quietly here (produce a date of 0/0/00), 
   // which will probably make your application angry.

   // ALSO NOTE: This has been optimized some to minimize redundant variables, with the side-effect
   // of making it much harder to understand. Please refer to the original appnote above
   // if you are trying to learn from it :-)


   //unsigned long hour;
   //unsigned long day;
   //unsigned long minute;
   //unsigned long second;
   unsigned long month;
   //unsigned long year;

	unsigned long seconds_left_2;
   //unsigned long whole_minutes;
   //unsigned long whole_hours;
   //unsigned long whole_days;
   //unsigned long whole_days_since_1968;
   unsigned long leap_year_periods;
   unsigned long days_since_current_lyear;
   //unsigned long whole_years;
   unsigned long days_since_first_of_year;
   unsigned long days_to_month;
   //unsigned long day_of_week;

   if(seconds_left >= 946684800)
   {
	   seconds_left -= 946684800; // correct for difference between DS1337 and UNIX epochs.

	   seconds_left_2 = seconds_left / 60; // seconds_left_2 = "whole_minutes"
	   rtc_bcd[DS1337_SEC] = bin2bcd(seconds_left - (60 * seconds_left_2));                 // leftover seconds

	   seconds_left = seconds_left_2 / 60; // seconds_left = "whole_hours"
	   rtc_bcd[DS1337_MIN] = bin2bcd(seconds_left_2 - (60 * seconds_left));            // leftover minutes

	   seconds_left_2 = seconds_left / 24; //seconds_left_2 = "whole_days"
	   rtc_bcd[DS1337_HR] = bin2bcd(seconds_left - (24 * seconds_left_2));         // leftover hours

	   //whole_days_since_1968 = whole_days;// + 365 + 366;	// seconds_left_2 = "whole_days" = "whole_days_since_1968"
	   leap_year_periods = seconds_left_2 / ((4 * 365) + 1);

	   days_since_current_lyear = seconds_left_2 % ((4 * 365) + 1);

	   // if days are after a current leap year then add a leap year period
	   if ((days_since_current_lyear >= (31 + 29))) {
		  leap_year_periods++;
	   }
	   seconds_left = (seconds_left_2 - leap_year_periods) / 365; // seconds_left = "whole_years"
	   days_since_first_of_year = seconds_left_2 - (seconds_left * 365) - leap_year_periods;

	   if ((days_since_current_lyear <= 365) && (days_since_current_lyear >= 60)) {
		  days_since_first_of_year++;
	   }
	   //year = seconds_left; // + 68;


		// seconds_left = "year"
		//seconds_left_2 = "month"
	   // walk across monthdays[] to find what month it is based on how many days have passed
	   //   within the current year
	   month = 13;
	   days_to_month = 366;
	   while (days_since_first_of_year < days_to_month) {
		   month--;
		   days_to_month = monthdays[month-1];
		   if ((month > 2) && ((seconds_left % 4) == 0)) {
			   days_to_month++;
			}
	   }
	   
	   rtc_bcd[DS1337_DATE] = bin2bcd( days_since_first_of_year - days_to_month + 1);

	   rtc_bcd[DS1337_DOW] = bin2bcd((seconds_left_2  + 4) % 7);


	   //rtc_bcd[DS1337_SEC] = bin2bcd(second);
	   //rtc_bcd[DS1337_MIN] = bin2bcd(minute);
	   //rtc_bcd[DS1337_HR] = bin2bcd(hour);
	   //rtc_bcd[DS1337_DATE] = bin2bcd(day);
	   //rtc_bcd[DS1337_DOW] = bin2bcd(day_of_week);
	   rtc_bcd[DS1337_MTH] = bin2bcd(month);
	   rtc_bcd[DS1337_YR] = bin2bcd(seconds_left);
   }
	else
	{
	// else: "invalid" (< year 2000) epoch format.
	// 'Best' way to handle this is to zero out the returned date. 
	
	   rtc_bcd[DS1337_SEC] = 0; //0x00 binary = 0x00 BCD
	   rtc_bcd[DS1337_MIN] = 0;
	   rtc_bcd[DS1337_HR] = 0;
	   rtc_bcd[DS1337_DATE] = 0;
	   rtc_bcd[DS1337_DOW] = 0;
	   rtc_bcd[DS1337_MTH] = 0;
	   rtc_bcd[DS1337_YR] = 0;
	}

}





void DS1337::snooze(unsigned long secondsToSnooze)
{ 
  // Given a value in secondsToSnooze, set an alarm for that many seconds into the future and go to sleep.
  // The alarm can be set for a maximum of 28-31 days into the future - it doesn't have settings for months or years.
  
  uint8_t sleep_reg_temp;
  
  readTime(); // update RTC library's buffers to contain the current time.
                  // Remember most functions (including epoch seconds stuff) work on what's in the buffer, not what's in the chip.

  
  setAlarmRepeat(EVERY_MONTH); // There is no DS1337 setting for 'alarm once' - once in a month is the most restrictive it gets.

  writeAlarm(date_to_epoch_seconds() + secondsToSnooze);
 
  attachInterrupt(RTC_INT_NUMBER, _dummy_int_handler, FALLING);  
  enable_interrupt();
  
  // the default snooze behavior is to put the CPU all the way to sleep. In case the user has previously set a different sleep mode,
  // save the entry sleep mode and restore it after sleeping. NOTE, set_sleep_mode() in avr/sleep.h is actually a giant device-specific mess
  // (making trying to implement a 'get_sleep_mode'-type function that works for all devices an equally nasty mess), but this should cover MOST
  // of the ones likely to be used with Arduino. For those others, user will have to (re)set the desired sleep mode by hand.
  
  #if defined(_SLEEP_CONTROL_REG)
  sleep_reg_temp = _SLEEP_CONTROL_REG;
  #endif
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  // enable deep sleeping
  sleep_enable();
  
  sleep_cpu(); // sleep. Will we waked by next alarm interrupt
 
  #if defined(_SLEEP_CONTROL_REG)
  _SLEEP_CONTROL_REG = sleep_reg_temp;
  #endif
  
  clear_interrupt(); // tell RTC to clear its interrupt flag and drop the INT line
  disable_interrupt(); // ensure we stop receiving interrupts
  detachInterrupt(RTC_INT_NUMBER); // disconnect INT2 from the current interrupt handler.
  
}

void DS1337::custom_snooze(unsigned long secondsToSnooze)
{ 
  // Same as snooze(), but do not change the current sleep mode. Use to sleep at a custom sleep mode other than ...PWR_DOWN.
  // Intentional use of a lighter sleep mode means the user is probably expecting/handling other interrupts - note of course that
  // most interrupts will wake the CPU from sleep mode, so the snooze may be shorter than specified in this case.
   
  readTime(); // update RTC library's buffers to contain the current time.
                  // Remember most functions (including epoch seconds stuff) work on what's in the buffer, not what's in the chip.

  
  setAlarmRepeat(EVERY_MONTH); // There is no DS1337 setting for 'alarm once' - once in a month is the most restrictive it gets.

  writeAlarm(date_to_epoch_seconds() + secondsToSnooze);
 
  attachInterrupt(RTC_INT_NUMBER, _dummy_int_handler, FALLING);  
  enable_interrupt();
  
  // enable deep sleeping
  sleep_enable();
  
  sleep_cpu(); // sleep. Will we waked by next alarm interrupt
 
  clear_interrupt(); // tell RTC to clear its interrupt flag and drop the INT line
  disable_interrupt(); // ensure we stop receiving interrupts
  detachInterrupt(RTC_INT_NUMBER); // disconnect INT2 from the current interrupt handler.
  
}

void DS1337::setSeconds(unsigned char v)
{
    rtc_bcd[DS1337_SEC] = bin2bcd(v);

}
void DS1337::setMinutes(unsigned char v)
{
    rtc_bcd[DS1337_MIN] = bin2bcd(v);

}
void DS1337::setHours(unsigned char v)
{
    rtc_bcd[DS1337_HR] = bin2bcd(v);

}
void DS1337::setDays(unsigned char v)
{
    rtc_bcd[DS1337_DATE] = bin2bcd(v);

}
void DS1337::setDayOfWeek(unsigned char v)
{
    rtc_bcd[DS1337_DOW] = bin2bcd(v);

}
void DS1337::setMonths(unsigned char v)
{
    rtc_bcd[DS1337_MTH] = bin2bcd(v);

}
void DS1337::setYears(unsigned int v)
{
    if (v>1999)
    {
        v -= 2000;
    }
    rtc_bcd[DS1337_YR] = bin2bcd(v);

}

byte DS1337::bcd2bin(byte v)
{
   return (v&0x0F) + ((v>>4)*10);
}

byte DS1337::bin2bcd(byte v)
{
   return ((v / 10)<<4) + (v % 10);
}

void DS1337::stop(void)
{
	setRegister(DS1337_SP, getRegister(DS1337_SP) | DS1337_SP_EOSC);
}

void DS1337::start(void)
{
	setRegister(DS1337_SP, getRegister(DS1337_SP) & !DS1337_SP_EOSC);
}
