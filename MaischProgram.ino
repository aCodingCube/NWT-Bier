// includes

// std
#include <Wire.h>
// lib
#include <LiquidCrystal_I2C.h>
// zip lib
#include <DallasTemperature.h>
#include <RCSwitch.h>
#include <PID_v1.h>

// setup OneWire
#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);

// setup TempSensor
DallasTemperature sensors(&oneWire);

// LCD
LiquidCrystal_I2C lcd(0x27,2,16);

// RCSwitch
RCSwitch sender = RCSwitch();

// define Enum
enum
{
  OFF  = 10359934, ON
};

// variables

// LCD framerate
unsigned long int timePoint1, timePoint2;

const unsigned long int timeIntervall = 1000;

bool lcdSwitch = true;

// PID
double wantedTemp;
double currentTemp;
double PIDresult;

PID PIDfast(&currentTemp,&PIDresult,&wantedTemp,1000,1,0,DIRECT);
PID PIDslow(&currentTemp,&PIDresult,&wantedTemp,400,1,0,DIRECT);

const unsigned long int highOutputLimit = 5000;

unsigned long int PIDtime;

// phases

const double degrees[3] = {52.0,62.0,72.0};
const unsigned int minutes[3] = {0,0,1};
unsigned long int milliseconds[3];
unsigned int instructionCount;

unsigned long int phaseTimePoint;
unsigned int phaseCounter = 0;


void setup() 
{
  // lcd init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Init LCD");
  delay(1000);
  lcd.clear();

  // sender initialized on digital Pin 2
  sender.enableTransmit(2);

  // temp
  sensors.begin();

  // init timePoint1
  timePoint1 = millis();

  // init PID
  PIDfast.SetOutputLimits(0,highOutputLimit);
  PIDfast.SetMode(AUTOMATIC); 
  PIDslow.SetOutputLimits(0,highOutputLimit);
  PIDslow.SetMode(AUTOMATIC);

  // sender
  sender.send(ON,24);

  // minutes to milliseconds
  instructionCount = (sizeof(degrees) / sizeof(double));
  phaseTimePoint = millis();
  for(int i = 0; i < instructionCount; i ++)
  {
    milliseconds[i] = minutes[i] * 60000;
  }

  // first phase
  wantedTemp = degrees[phaseCounter];

  // Serial Monitor for debugging
  Serial.begin(9600);

  // innit PIDtime
  PIDtime = millis();
}


// define Lambda
auto temp = []()
  ->double
  {
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    return static_cast<double>(temp);
  };

void loop() 
{
  // update Temp
  currentTemp = temp();

  // update PID
  TempController();
  
  // update phase schedule
  if((millis() - phaseTimePoint) > milliseconds[phaseCounter])
  {
    phaseCounter ++;
    wantedTemp = degrees[phaseCounter];
    phaseTimePoint = millis();
  }

  // lcd update event
  if((millis()-timePoint1) > timeIntervall)
  {
    timePoint1 = millis();
    PrintToLCD();
  }
  Serial.print("[DEBUG] PID-Result: ");
  Serial.println(PIDresult);
}

void TempController()
{
  if(temp() <= (wantedTemp - 8)) // PID fast
  {
    PIDfast.Compute();
    HeatingController(); // adjust heating
  }
  else // PID slow
  {
    PIDslow.Compute();
    HeatingController(); // adjust heating
  }
}

void HeatingController()
{
  if((millis() - PIDtime) > PIDresult) // if time > wanted time
  {
    sender.send(OFF,24); // turn heating off
  }
  else if((millis() - PIDtime) > highOutputLimit) // if timeframe is over
  {
    PIDtime = millis();
    sender.send(ON,24); // turn heating on
    
  }

  if(phaseCounter > (instructionCount-1)) // when there are no more instructions
  {
     sender.send(OFF,24); // turn off heating
     lcd.clear();
     lcd.print("Finished all instructions!");
     wantedTemp = 0;
     delay(10000);
  }

}

void PrintToLCD()
{
  // clear
  lcd.clear();
  // times
  if(lcdSwitch)
  {
    lcdSwitch = false;
    lcd.setCursor(1,0);
    //lcd.print((millis()-phaseTimePoint) + " >  " + minutes[phaseCounter]);
    lcd.print((millis()-phaseTimePoint)/60000);
    lcd.print(" >  ");
    lcd.print(minutes[phaseCounter]);
    lcd.setCursor(1,1);
    //lcd.print(temp() + " >  " + degrees[phaseCounter]);
    lcd.print(temp());
    lcd.print(" >  ");
    lcd.print(degrees[phaseCounter]);
  }
  else
  {
    lcdSwitch = true;
    lcd.setCursor(1,0);
    //lcd.print((millis()-phaseTimePoint) + "  > " + minutes[phaseCounter]);
    lcd.print((millis()-phaseTimePoint)/60000);
    lcd.print("  > ");
    lcd.print(minutes[phaseCounter]);
    lcd.setCursor(1,1);
    //lcd.print(temp() + "  > " + degrees[phaseCounter]);
    lcd.print(temp());
    lcd.print("  > ");
    lcd.print(degrees[phaseCounter]);
  }
}
