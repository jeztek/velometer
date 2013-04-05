#include <EEPROM.h>
#include <Wire.h>
#include <DS1337.h>
#include "TEEPROM.h"

#define LED_PIN                 13

#define PRESSURE_SENSOR         3
#define PRESSURE_THRESHOLD      2

#define TIMEOUT_MILLIS          2000

#define EEPROM_COUNTER_ADDR     0
#define EEPROM_TIME_ADDR        4
#define EEPROM_SAVE_SECONDS     300

#define DISP_ADDR		0x50 // (01010000b - AD0, AD1 set to ground)
#define DISP_MAX_NUMBER         9999

#define DISP_INTENSITY10_CMD	0x01
#define DISP_INTENSITY32_CMD	0x02
#define DISP_SCANLIMIT_CMD	0x03
#define DISP_CONFIG_CMD		0x04
#define DISP_USERFONT_CMD	0x05
#define DISP_TEST_CMD		0x07

#define DISP_DIGIT0_P0		0x20
#define DISP_DIGIT1_P0		0x21
#define DISP_DIGIT2_P0		0x22
#define DISP_DIGIT3_P0		0x23

#define DISP_DIGIT0_P1		0x40
#define DISP_DIGIT1_P1		0x41
#define DISP_DIGIT2_P1		0x42
#define DISP_DIGIT3_P1		0x43

DS1337 RTC = DS1337();

volatile int secondsCounter = 0;
int displayTimeMode = false;
int state = 0;

volatile boolean runTimely = false;
unsigned int numBikes;

unsigned long firstTireTime;
unsigned long secondTireTime;
int pressureInit;      // baseline pressure value
int pressureVal;       // latest pressure reading
int pressureDelta;
int pressureMax;

void displayInit()
{
  Wire.begin();
  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_CONFIG_CMD);
  Wire.write(0x05);  // 00000101b - normal operation, fast blink
  Wire.endTransmission();
}

// Set display intensity, each nibble corresponds to a digita
// and can range from 0x01 to 0x0E
// Full intensity is 0xEE, 0xEE
void displaySetIntensity(byte val32, byte val10)
{
  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_INTENSITY10_CMD);
  Wire.write(val10);
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_INTENSITY32_CMD);
  Wire.write(val32);
  Wire.endTransmission();
}

void displayHasFourDigits(boolean val)
{
  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_SCANLIMIT_CMD);
  if (val)
    Wire.write(0x01);
  else
    Wire.write(0x00);
  Wire.endTransmission();
}

void displayInitUserFont()
{
  // User defined digit
  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_USERFONT_CMD);
  Wire.write(0x80);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);  
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_USERFONT_CMD);
  Wire.write(0x85);
  Wire.write(0x7F);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);  
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_USERFONT_CMD);
  Wire.write(0x8A);
  Wire.write(0x7F);
  Wire.write(0x7F);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);  
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_USERFONT_CMD);
  Wire.write(0x8F);
  Wire.write(0x7F);
  Wire.write(0x7F);
  Wire.write(0x7F);
  Wire.write(0x00);
  Wire.write(0x00);  
  Wire.endTransmission();
}

void displayEnableTest(boolean val)
{
  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_TEST_CMD);
  if (val)
    Wire.write(0x01);
  else
    Wire.write(0x01);
  Wire.endTransmission();
}

void displayWriteNumber(unsigned int value)
{
  char digits[5];

  // We can't display values greater than the number of digits available
  if (value > DISP_MAX_NUMBER) {
    value = DISP_MAX_NUMBER;  
  }
  
  // Right justify numbers
  utoa(value, digits, 10);
  if (value < 10) {
    digits[3] = digits[0];    
    digits[2] = 0x20;    
    digits[1] = 0x20;    
    digits[0] = 0x20;
  }
  else if (value < 100) {
    digits[3] = digits[1];    
    digits[2] = digits[0];    
    digits[1] = 0x20;    
    digits[0] = 0x20;     
  }
  else if (value < 1000) {
    digits[3] = digits[2];    
    digits[2] = digits[1];    
    digits[1] = digits[0];    
    digits[0] = 0x20;         
  }
  displayWriteChars(digits);
}

void displayWriteChars(char *str)
{
  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_DIGIT0_P0);
  Wire.write(str[0]);
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_DIGIT1_P0);
  Wire.write(str[1]);
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_DIGIT2_P0);
  Wire.write(str[2]);
  Wire.endTransmission();

  Wire.beginTransmission(DISP_ADDR);
  Wire.write(DISP_DIGIT3_P0);
  Wire.write(str[3]);
  Wire.endTransmission();  
}


void displayWriteChar(char val, int digitNum)
{
  if (digitNum >= 0 && digitNum <= 3) {
    Wire.beginTransmission(DISP_ADDR);
    Wire.write((DISP_DIGIT0_P0 + digitNum) & 0xFF);
    Wire.write(val);
    Wire.endTransmission();  
  }
}

int pressureSensorInit()
{
   int pressureAvg = 0;
   int i;
   
   for (i = 0; i < 5; i++)
   {
     pressureAvg += analogRead(PRESSURE_SENSOR);
     delay(50);  
   }
   pressureAvg = pressureAvg / 5;
   return pressureAvg;
}

void isr()
{
  secondsCounter += 1;
  runTimely = true;
}

void printMenu()
{
  Serial.println("\n=== User Menu ===");
  Serial.print("Num bikes: ");
  Serial.println(numBikes);
  Serial.println("1 - Reset counter");  
  Serial.println("2 - Set counter value");
  Serial.println("3 - Save counter to EEPROM");
  Serial.println("4 - Seconds to next counter save");
  Serial.println("5 - Get time");
  Serial.println("6 - Set time");
  Serial.flush();  
}

void rtcPrintTime()
{
  RTC.readTime();
  Serial.print(int(RTC.getMonths()));
  Serial.print("/");
  Serial.print(int(RTC.getDays()));
  Serial.print("/");
  Serial.print(RTC.getYears());
  Serial.print(" ");
  Serial.print(int(RTC.getHours()));
  Serial.print(":");  
  Serial.print(int(RTC.getMinutes()));
  Serial.print(":");  
  Serial.println(int(RTC.getSeconds()));
}

void timely()
{
  // Reset counter at midnight
  RTC.readTime();
  int month = int(RTC.getMonths());
  int days = int(RTC.getDays());
  int hours = int(RTC.getHours());
  int mins = int(RTC.getMinutes());
  int secs = int(RTC.getSeconds());
  
  if (hours == 0 && mins == 0 && secs == 0) {
    numBikes = 0;  
  }

  // Save counter data to EEPROM every EEPROM_SAVE_SECONDS
  if (secondsCounter >= EEPROM_SAVE_SECONDS) {
    int time = month * 100 + days;    
    TEEPROM_write(EEPROM_COUNTER_ADDR, numBikes);
    TEEPROM_write(EEPROM_TIME_ADDR, time); 
    secondsCounter = 0;
    Serial.println("Saved counter to EEPROM");
  }
  

}

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing Velometer...");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize clock and configure 1Hz interrupt trigger
  RTC.start();
  RTC.setRegister(DS1337_SP, RTC.getRegister(DS1337_SP) & !DS1337_SP_RS2 & !DS1337_SP_RS1);
  attachInterrupt(1, isr, RISING);
  Serial.println("  [OK] RTC");
  
  // Initialize display
  displayInit();
  displaySetIntensity(0x33, 0x33);
  displayHasFourDigits(true);
  displayWriteChars("Helo");
  Serial.println("  [OK] display");
  
  // Initialize pressure sensor
  pressureInit = pressureSensorInit();
  pressureMax = 0;
  Serial.print("  [OK] pressure sensor, init: ");  
  Serial.println(pressureInit);

  // Initialize counter and update display
  int memday, curday;
  RTC.readTime();
  int month = int(RTC.getMonths());
  int days = int(RTC.getDays());
  curday = month * 100 + days;
  TEEPROM_read(EEPROM_TIME_ADDR, memday);  
  if (curday > memday) {
    numBikes = 0; 
  }
  else {
    TEEPROM_read(EEPROM_COUNTER_ADDR, numBikes);
  }
  displayWriteNumber(numBikes);
  Serial.print("  [OK] EEPROM, memday: ");
  Serial.print(memday);
  Serial.print(" curday: ");
  Serial.print(curday);
  Serial.print(" num bikes: ");
  Serial.println(numBikes);
  
  Serial.println("Ready!");
  printMenu();
}

void loop()
{
  if (runTimely == true) {
    timely();
    runTimely = false;
  }
  
  pressureVal = analogRead(PRESSURE_SENSOR);
  pressureDelta = pressureVal - pressureInit;
  
  switch(state) {
    case 0:
      pressureMax = 0;
      
      // First tire on tube
      if (pressureDelta > PRESSURE_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);
        Serial.print("tire 1 ");
        state = 1;
      }
      break;

    case 1:
      if (pressureVal > pressureMax) 
      {
        pressureMax = pressureVal;
      }
      
      // First tire off tube
      if (pressureDelta <= 0) {
        firstTireTime = millis();

        digitalWrite(LED_PIN, LOW);
        Serial.print("pres: ");
        Serial.print(pressureInit);
        Serial.print("/");
        Serial.println(pressureMax);
        pressureMax = 0;
        state = 2;
      }
      break;
      
    case 2:
      // Timeout
      if ((millis() - firstTireTime) > TIMEOUT_MILLIS) {
        digitalWrite(LED_PIN, LOW);
        Serial.println("timeout");
        state = 0;
      }
      
      // Second tire on tube
      else if (pressureDelta > PRESSURE_THRESHOLD) {
        digitalWrite(LED_PIN, HIGH);
        Serial.print("tire 2 ");
        state = 3; 
      }
      break;
      
    case 3:
      if (pressureMax < pressureVal) {
        pressureMax = pressureVal;
      }
      
      // Timeout
      if ((millis() - firstTireTime) > TIMEOUT_MILLIS) {
        digitalWrite(LED_PIN, LOW);
        Serial.println("timeout");
        state = 0;
      }
      
      // Second tire off tube
      else if (pressureDelta <= 0) {
        secondTireTime = millis();

        digitalWrite(LED_PIN, LOW);
        Serial.print("pres: ");
        Serial.print(pressureInit);
        Serial.print("/");
        Serial.print(pressureMax);
        pressureMax = 0;
        
        // TODO: if time interval is good, increment counter
        numBikes += 1;
        displayWriteNumber(numBikes);

        Serial.print(", num bikes: ");
        Serial.println(numBikes);

        state = 0;
      }
      break;  
  }
  
  if (Serial.available() > 0) {
    byte input = Serial.read();
    
    // Reset counter
    if (input == '1') {
      numBikes = 0;
      TEEPROM_write(EEPROM_COUNTER_ADDR, numBikes);
      Serial.println("Counter reset, num bikes: 0");
    }
    
    // Set counter
    else if (input == '2') {
      Serial.println("Enter counter value and press enter");
      Serial.setTimeout(100000);
      int val = Serial.parseInt();
      numBikes = val;
      displayWriteNumber(numBikes);
    }

    // Save counter to EEPROM
    else if (input == '3') {
      RTC.readTime();
      int month = int(RTC.getMonths());
      int days = int(RTC.getDays());
      int curday = month * 100 + days;
      TEEPROM_write(EEPROM_COUNTER_ADDR, numBikes);
      TEEPROM_write(EEPROM_TIME_ADDR, curday);
      Serial.print("Num bikes: ");
      Serial.println(numBikes);
      Serial.print("Day: ");
      Serial.println(curday);
    }
    
    // Seconds remaining to EEPROM save
    else if (input == '4') {
      int remainSeconds = EEPROM_SAVE_SECONDS - secondsCounter;
      Serial.print("Seconds remaining: ");
      Serial.println(remainSeconds);  
    }
    
    // Get time
    else if (input == '5') {
      Serial.println("Current time:");
      rtcPrintTime();      
    }
    
    // Set time
    else if (input == '6') {
      Serial.println("Enter time in the following format: \"mm dd yyyy hh mm\" and press enter");

      int mo, dd, yyyy, hh, mm;
      Serial.setTimeout(100000);
      mo = Serial.parseInt();
      dd = Serial.parseInt();
      yyyy = Serial.parseInt();
      hh = Serial.parseInt();
      mm = Serial.parseInt();
      
      char next = Serial.read();
      if (next == '\n' || next == '\r') {
        RTC.setSeconds(0);
        RTC.setMinutes(mm);
        RTC.setHours(hh);
        RTC.setDays(dd);
        RTC.setMonths(mo);
        RTC.setYears(yyyy);
        RTC.writeTime();
        Serial.println("Time set to:");
        rtcPrintTime();
      }
    }  
    else if (input == '7')
    {
      displayTimeMode = !displayTimeMode;
    } 
    else if (input == '\n' || input == '\r')
    {
      printMenu();
    }
  }
}

