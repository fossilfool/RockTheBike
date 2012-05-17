/**** Split-rail Pedalometer
 * Arduino code to run the sLEDgehammer
 * ver. 1.7
 * Written by:
 * Thomas Spellman <thomas@thosmos.com>
 * Jake <jake@spaz.org>
 * Paul@rockthebike.com
 * Copyright: 2012, Rock the Bike (http://rockthebike.com)
 * License: This code is distributed under the GPL license: http://www.gnu.org/licenses/gpl.html
 * 
 * 1.6 -- First version with Wattage output display. 
 * 1.7 -- cleaned up serial output, six LED pedalometer, removed whichPin[], inverted minusRelay state (minus relay is wired normally-open now), re-inits display every minute,  [jake]
 */

const char versionStr[] = "Split Rail Pedal Power Utility Box ver. 1.7 . Features include Wattage output display";

/*Buglist: wattage changes while being displayed.
JBL / Watt switches too frequently. 

*/

/*

 Check the system voltage. 
 Use a double array to check desired behavior for current voltage.
 Do the desired behavior until the next check.
 
 Repeat. 
 
 */
#include <Wire.h>
#include <avr/pgmspace.h>
#include "ht1632c.h"
#include "font1.h"

const int pwm = 0;
const int onoff = 1;

const int numLevels = 6;

// Arduino pin for each output level
const int numPins = 6; // Number of active Arduino Pins for + rail team.
int pin[numPins] = {6, 11, 5, 9, 3, 10}; 
// above pinout is for Rock The Bike box. Use this for 4-row display: {3, 5, 6, 9};

// voltages at which to turn on each level
//float levelVolt[numLevels] = {21, 24.0, 27.0};
float levelVolt[numLevels] = {
  22.0, 23.5, 24.8, 25.7, 26.7, 27.2};
  
int levelMode=0; // 0 = off, 1 = blink, 2 = steady

// voltages at which to turn on each level
//float levelVolt[numLevels] = {21, 24.0, 27.0};
//float minusRailLevelVolt[numLevels] = {22.0, 23.5, 24.8, 26.7, 27.2};
//int minusRailLevelMode=0; // 0 = off, 1 = blink, 2 = steady

int levelType[numLevels] = {pwm, pwm, pwm, pwm, pwm, pwm};


// type of each level (pwm or onoff)

// Arduino pins to be used as inputs to voltage sensor. This shows where to connect the wires! 
const int voltPin = A0; // Voltage Sensor Input
const int minusVoltPin = A3; // Voltage Sensor Input
const int ACAmpsPin = A2; // AC power and all + rail DC devices. 
const int plusRailAmpsPin = A1; // Plus rail of JBLs only. 

const int relayPin=8;
 
/*
Pin A0 --- AttoPilot "V"
 Pin A1 --- AttoPilot "I"
 GND ------ AttoPilot "GND"
 GND ------ SerLCD "GND"
 5V ------- SerLCD "VCC"
 */

const int minusRelayPin=2;

//adc = v * 10/110/5 * 1024 == v * 18.618181818181818; // for a 10k and 100k voltage divider into a 10bit (1024) ADC that reads from to 5V

const float voltcoeff = 13.25;  // larger numer interprets as lower voltage 
const float ampcoeff = 7.4; //One direct calculation has this at 5.8. Tune. 

//MAXIMUM VOLTAGE TO GIVE LEDS
//const float maxVoltLEDs = 24 -- always run LEDs in 24V series configuration.
const float maxVoltLEDs = 21.0; //LED
const float maxVoltPlusRail = 27.4;
const float maxVoltMinusRail = 25;

//Hysteresis variables
const float minusRailComeBackInVoltage = 22.5;
const float plusRailComeBackInVoltage = 26;
const float plusRailComeBackInVoltagetwelveVoltMode = 13.7;
int plusRailHysteresis=0;
int minusRailHysteresis=0;

// vars to store temp values
int adcvalue = 0;

//Voltage related variables. 
float voltage = 0;
float minusRailVoltage=0;

//Current related variables
float plusRailAmps=0;
float ACAmps=0; 
int plusRailAmpsRaw;
int ACAmpsRaw;

float ACWatts;
float plusRailWatts;
float wattage;
// on/off state of each level (for use in status output)
int state[numLevels];
int desiredState[numPins];

const int AVG_CYCLES = 50; // average measured voltage over this many samples
const int DISPLAY_INTERVAL_MS = 500; // when auto-display is on, display every this many milli-seconds
int displayCount = 0;  // counts how many display intervals have happened
const int resetInterval = 120; // how many display intervals between display resets 
const int VICTORYTIME=4000;

int readCount = 0; // for determining how many sample cycle occur per display interval

// vars for current PWM duty cycle
int pwmValue;
//int pwmvalue24V;
boolean updatePwm = false;

////Kindermode variables
//boolean twelveVoltPedalometerMode = false;
//boolean twelveVoltProtectionMode = false;
//const float maxVoltTwelveVoltPedalometerMode = 14.5;


// timing variables for blink. Mark, there are too many of these. 
int blinkState=0;
int fastBlinkState=0;
unsigned long time = 0;
unsigned long currentTime = 0;
unsigned long lastFastBlinkTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long timeRead = 0;
unsigned long timeDisplay = 0;
unsigned long timeWeStartedCheckingForVictory = 0;

// Time related variables. Mark, this may be a vestige of the sLEDgehammer code but we should bring this feature back because in this
//video, the flickering would be prevented:
//Basically this is a minimum time to keep each level on for before you can change it. http://www.rockthebike.com/pedal-power-utility
//box-from-rock-the-bike/  
unsigned long onTime = 500;
unsigned long levelTime = 0;

// current display level
int level = -1;

// var for looping through arrays
int i = 0;
int x = 0;
int y = 0;


//About to add a bunch of variables related to the ht1632 display. 

//#include <Wire.h>
//#include <avr/pgmspace.h>
//
//#include "ht1632c.h"
//#include "font1.h"

// Dual 3216 LED Matrix
// If you have only one set these to:
// X_MAX=31
// Y_MAX=15
// CHIP_MAX=4
#define X_MAX 63 // 0 based X
#define Y_MAX 15 // 0 based Y
#define CHIP_MAX 4*2 // Number of HT1632C Chips
// 4 each board * 2 boards

// possible values for a pixel;
#define BLACK  0
#define GREEN  1
#define RED    2
#define ORANGE 3

#define ASSERT(condition) //nothing

/*
 * Set these constants to the values of the pins connected to the SureElectronics Module
 */
//this new pinout corresponds to the first two-team maker-faire arbduino setup
static const byte ht1632_cs = 7;    // Chip Select (pin 1)
static const byte ht1632_clk = 4;  // Clk pin (pin 2)
static const byte ht1632_wrclk = 12; // Write clock pin (pin 5)
static const byte ht1632_data = 13;  // Data pin (pin 7)
// The should also be a common GND (pins .
// The module with all LEDs like draws about 200mA,
//  which makes it PROBABLY powerable via Arduino +5V

/*
 * we keep a copy of the display controller contents so that we can
 * know which bits are on without having to (slowly) read the device.
 * Note that we only use the low four bits of the shadow ram, since
 * we're shadowing 4-bit memory.  This makes things faster, but we
 * COULD do something with the other half of our bytes !
 */
byte ht1632_shadowram[63][CHIP_MAX] = {
  0};
/*
 * we use buffer to put the char fonts
 */
char buffer[8][8];

//**************************************************************************************************
//Function Name: decToBcd
//Function Feature: Convert normal decimal numbers to binary coded decimal
//Input Argument: void
//Output Argument: void
//**************************************************************************************************



void setup() {
  Serial.begin(57600);
  ht1632_initialize();

  // Initialize Software PWM
  //SoftPWMBegin();

  Serial.println(versionStr);

  pinMode(voltPin,INPUT);
  pinMode(minusVoltPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(minusRelayPin, OUTPUT);
  
  // init LED pins
  for(i = 0; i < numLevels; i++) {
    pinMode(pin[i],OUTPUT);
    if(levelType[i] == pwm)
      analogWrite(pin[i],0);
    else if(levelType[i] == onoff)
      digitalWrite(pin[i],0);
  }

  //init - Rail LED / pedalometer pins
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);

  digitalWrite(minusRelayPin, HIGH);  // MINUS RELAY IS WIRED FOR NORMALLY-OPEN !!!
}

boolean levelLock = false;
int senseLevel = -1;

void loop() {

  //   digitalWrite(minusRelayPin, LOW);
  //   Serial.println("Trying to protect.");
  //  delay(2000);
  //   digitalWrite(minusRelayPin, HIGH);
  //   Serial.println("Trying to connect.");
  //delay(2000);

  getVoltages();
  getCurrents();
  setpwmvalue();
  readCount++;


  //First deal with the blink  

  time = millis();
  currentTime=millis();
  if (((currentTime - lastBlinkTime) > 600) && blinkState==1){
    //                  Serial.println("I just turned pwm to 0.");
    //     pwmValue=0;
    blinkState=0;
    //   analogWrite(pin[i], pwmValue);
    lastBlinkTime=currentTime;
  } 
  else if (((currentTime - lastBlinkTime) > 600) && blinkState==0){
    //   Serial.println("I just turned blinkstate to 1.");
    blinkState=1; 
    lastBlinkTime=currentTime;
  }


  if (((currentTime - lastFastBlinkTime) > 120) && fastBlinkState==1){
    //                  Serial.println("I just turned pwm to 0.");
    //     pwmValue=0;
    fastBlinkState=0;
    //   analogWrite(pin[i], pwmValue);
    lastFastBlinkTime=currentTime;
  } 
  else if (((currentTime - lastFastBlinkTime) > 120) && fastBlinkState==0){
    //   Serial.println("I just turned blinkstate to 1.");
    fastBlinkState=1; 
    lastFastBlinkTime=currentTime;
  }  

  //protect the ultracapacitors if needed
  if (voltage > maxVoltPlusRail){
    digitalWrite(relayPin, HIGH); 
    plusRailHysteresis=1;
  }

  if (minusRailVoltage > maxVoltMinusRail){
    digitalWrite(minusRelayPin, LOW); // MINUS RELAY IS WIRED FOR NORMALLY-OPEN !!!
    minusRailHysteresis=1;

  }


  if (plusRailHysteresis==1 && voltage < plusRailComeBackInVoltage){
    digitalWrite(relayPin, LOW);
    plusRailHysteresis=0;
  } 

  if (minusRailHysteresis==1 && minusRailVoltage < minusRailComeBackInVoltage){
    digitalWrite(minusRelayPin, HIGH);  // MINUS RELAY IS WIRED FOR NORMALLY-OPEN !!!
    minusRailHysteresis=0;
  }
  // } 

  //Set the desired lighting states. 

  senseLevel = -1;
  if (voltage <=levelVolt[0]){
    senseLevel=0;
    //    Serial.println("Level Mode = 1");
    desiredState[0]=1;
  } 
  else {

    for(i = 0; i < numLevels; i++) {

      if(voltage >= levelVolt[i]){
        senseLevel = i;
        desiredState[i]=2;
        levelMode=2;
      } 
      else desiredState[i]=0;
    }
  }

  level=senseLevel;
  //    Serial.print("Level: ");
  //   Serial.println(level);
  if (level == (numLevels-1)){
    // Serial.print("LevelMode = 1");
    desiredState[level] = 3; //workaround
  }

  // End setting desired states. 



  // Do the desired states. 
  // loop through each led and turn on/off or adjust PWM



  for(i = 0; i < numPins; i++) {

    if(levelType[i]==pwm) {

      if(desiredState[i]==2){
        analogWrite(pin[i], pwmValue);

        state[i] = 2;
      }
      else if (desiredState[i]==0){
        analogWrite(pin[i], 0);
        state[i] = 0;
      }
      else if (desiredState[i]==1 && blinkState==1){
        analogWrite(pin[i], pwmValue);
        state[i] = 1;
      }
      else if (desiredState[i]==1 && blinkState==0){
        analogWrite(pin[i], 0);
        state[i] = 1;
      }
      else if (desiredState[i]==3 && fastBlinkState==1){
        analogWrite(pin[i], pwmValue);
        state[i] = 1;
      }
      else if (desiredState[i]==3 && fastBlinkState==0){
        analogWrite(pin[i], 0);
        state[i] = 1; 

      }
    } 
  } //end for

  //Now show the - Team how hard to pedal. 

  //    if (minusRailVoltage > 19){
  //       analogWrite(11, pwmValue);
  //       analogWrite(10, LOW);
  //       Serial.println ("Turning - rail greens on"); 
  //    } 

  //    if (minusRailVoltage <= 19){
  //       analogWrite(10, pwmValue);
  //      analogWrite(11, LOW); 
  //       Serial.println ("Turning - rail reds on");       
  //    }    


  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    displayCount += 1; //increment displayCount counter
    if (displayCount > resetInterval) {
      displayCount = 0;
        ht1632_initialize();
    }
    printDisplay();
    readCount = 0;
  }

  // Adding HT1632 code: 

  char* label;

  if( time % 2000 > 1000 ) {  // display total wattage
  // only calculate wattage once per display update
    if (label != "WATT") wattage = voltage * ACAmps + voltage * plusRailAmps + minusRailVoltage * plusRailAmps; //Assuming - rail amps = + rail amps. 
    label = "WATT";
  } 
  else {  // display jbl wattage
  // only calculate wattage once per display update
    if (label != "JBL ") wattage = voltage * plusRailAmps + minusRailVoltage * plusRailAmps; //Assuming - rail amps = + rail amps. 
    label = "JBL ";
  }
  char buf[]="    ";
  sprintf(buf, "%4d", (int)wattage);
  ht1632_draw_strings( label, buf );

  delay(200);


}

void setpwmvalue()
{

  // The effective voltage of a square wave is
  // Veff = Vin * sqrt(k)
  // where k = t/T is the duty cycle

    // If you want to make 12V from a 20V input you have to use
  // k = (Veff/Vin)^2 = (12/20)^2 = 0.36

  int newVal = 0;

  if (voltage <= maxVoltLEDs) {
    newVal = 255.0;
  }
  else {
    newVal = maxVoltLEDs / voltage * 255.0;
  }

  // if(voltage <= 24) {
  // pwmvalue24V = 255.0;
  // }
  // else {
  // pwmvalue24V = sq(24 / voltage) * 255.0;
  // }

  if(newVal != pwmValue) {
    pwmValue = newVal;
    updatePwm = true;
  }
}

void getCurrents(){

  //first two lines are for AC amps 
  ACAmpsRaw = analogRead(ACAmpsPin);
  //Serial.print("AC Amps Raw: ");
  //Serial.println(ACAmpsRaw);
  // adcvalue = analogRead(ACAmpsPin);
  //  ACAmps = average(adc2amps(adcvalue), ACAmps);
  ACAmps = adc2amps(ACAmpsRaw);  // constant multiplier needed

  //next two lines are for Plus Rail amps 
  plusRailAmpsRaw = analogRead(plusRailAmpsPin);
  //  Serial.print("Plus Rail Amps Raw: ");
  //Serial.println(plusRailAmpsRaw);
  // plusRailAmps = average(adc2amps(adcvalue), plusRailAmps); //Mark we might want to average this after all but commenting it out to 
//  et program up and running
    //  minusRailVoltage = average(adc2volts(adcvalue), minusRailVoltage) - 5;
  plusRailAmps = adc2amps(plusRailAmpsRaw); // constant multiplier needed.
}

void getVoltages(){

  //first two lines are for + rail
  adcvalue = analogRead(voltPin);
  voltage = average(adc2volts(adcvalue), voltage);

  // Here down is to measure - rail
  adcvalue = 1023 - analogRead(minusVoltPin) ;

  minusRailVoltage = adc2volts(adcvalue) - 5;

  //  minusRailVoltage = average(adc2volts(adcvalue), minusRailVoltage) - 5;

}

void protectUltracap(int whichRail){

  // Turn OFF the relay to protect either the minus or the plus rail 

}

void restoreUltracapAfterHysteresis(){
  // Turn ON the relay for normal operation
}

float average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

static int volts2adc(float v){
  /* voltage calculations
   *
   * Vout = Vin * R2/(R1+R2), where R1 = 100k, R2 = 10K
   * 30V * 10k/110k = 2.72V // at ADC input, for a 55V max input range
   *
   * Val = Vout / 5V max * 1024 adc val max (2^10 = 1024 max vaue for a 10bit ADC)
   * 2.727/5 * 1024 = 558.4896
   */
  //int led3volts0 = 559;

  /* 24v
   * 24v * 10k/110k = 2.181818181818182
   * 2.1818/5 * 1024 = 446.836363636363636
   */
  //int led2volts4 = 447;

  //adc = v * 10/110/5 * 1024 == v * 18.618181818181818;

  return v * voltcoeff;
}



float adc2volts(float adc){
  // v = adc * 110/10 * 5 / 1024 == adc * 0.0537109375;
  return adc * (1 / voltcoeff); // 55 / 1024 = 0.0537109375;
}

float adc2amps(float adc){

  return adc * (1 / ampcoeff); 
}


void printDisplay(){
  Serial.print("plusrail volts: ");
  Serial.print(voltage);
  if (plusRailHysteresis) Serial.print(" PROTECTED ");
  Serial.print(", AC Amps: ");
  Serial.print(ACAmps);
  Serial.print(", Minus volts: ");
  Serial.print(minusRailVoltage);
  if (minusRailHysteresis) Serial.print(" PROTECTED ");
  //  Serial.print(", pwm value: ");
  //  Serial.print(pwmValue);
  Serial.print(", Levels ");
  for(i = 0; i < numLevels; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(desiredState[i]);
    Serial.print(", ");
  }

  Serial.println("");

  Serial.print("Total wattage is ");
  Serial.println(voltage * ACAmps + voltage * plusRailAmps + minusRailVoltage * plusRailAmps);
  Serial.print("JBL wattage is ");
  Serial.println(voltage * plusRailAmps + minusRailVoltage * plusRailAmps);


  //  
  //    Serial.print("Desired State ");
  //  
  //  for(i = 0; i < numLevels; i++) {
  //    Serial.print(i);
  //    Serial.print(": ");
  //    Serial.print(desiredState[i]);
  //    Serial.print(", ");
  //  }
  Serial.println();

}


byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

//**************************************************************************************************
//Function Name: bcdToDec
//Function Feature: Convert binary coded decimal to normal decimal numbers
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

//**************************************************************************************************
//Function Name: OutputCLK_Pulse
//Function Feature: enable CLK_74164 pin to output a clock pulse
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
void OutputCLK_Pulse(void)	//Output a clock pulse
{
  digitalWrite(ht1632_clk, HIGH);
  digitalWrite(ht1632_clk, LOW);
}

//**************************************************************************************************
//Function Name: ht1632_chipselect
//Function Feature: enable HT1632C
//Input Argument: select: HT1632C to be selected
//	If select=0, select none.
//	If s<0, select all.
//Output Argument: void
//**************************************************************************************************
void ht1632_chipselect(int select)
{
  unsigned char tmp = 0;
  if (select < 0) { // Enable all HT1632C
    digitalWrite(ht1632_cs, LOW);
    for (tmp=0; tmp<CHIP_MAX; tmp++) {
      OutputCLK_Pulse();
    }
  } 
  else if(select==0) { //Disable all HT1632Cs
    digitalWrite(ht1632_cs, HIGH);
    for(tmp=0; tmp<CHIP_MAX; tmp++) {
      OutputCLK_Pulse();
    }
  } 
  else {
    digitalWrite(ht1632_cs, HIGH);
    for(tmp=0; tmp<CHIP_MAX; tmp++) {
      OutputCLK_Pulse();
    }
    digitalWrite(ht1632_cs, LOW);
    OutputCLK_Pulse();
    digitalWrite(ht1632_cs, HIGH);
    tmp = 1;
    for( ; tmp<select; tmp++) {
      OutputCLK_Pulse();
    }
  }
}

//**************************************************************************************************
//Function Name: ht1632_writebits
//Function Feature: Write bits (up to 8) to h1632 on pins ht1632_data, ht1632_wrclk
//                  Chip is assumed to already be chip-selected
//                  Bits are shifted out from MSB to LSB, with the first bit sent
//                  being (bits & firstbit), shifted till firsbit is zero.
//Input Argument: bits: bits to send
//	      firstbit: the first bit to send
//Output Argument: void
//**************************************************************************************************
void ht1632_writebits (byte bits, byte firstbit)
{
  DEBUGPRINT(" ");
  while (firstbit) {
    DEBUGPRINT((bits&firstbit ? "1" : "0"));
    digitalWrite(ht1632_wrclk, LOW);
    if (bits & firstbit) {
      digitalWrite(ht1632_data, HIGH);
    }
    else {
      digitalWrite(ht1632_data, LOW);
    }
    digitalWrite(ht1632_wrclk, HIGH);
    firstbit >>= 1;
  }
}

//**************************************************************************************************
//Function Name: ht1632_sendcmd
//Function Feature: Send a command to the ht1632 chip.
//                  Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx Free
//Input Argument: chipNo: the chip you want to send data
//               command: consists of a 3-bit "CMD" ID, an 8bit command, and
//                        one "don't care bit".
//Output Argument: void
//**************************************************************************************************
static void ht1632_sendcmd (byte chipNo, byte command)
{
  ht1632_chipselect(chipNo);
  ht1632_writebits(HT1632_ID_CMD, 1<<2);  // send 3 bits of id: COMMMAND
  ht1632_writebits(command, 1<<7);  // send the actual command
  ht1632_writebits(0, 1); 	/* one extra dont-care bit in commands. */
  ht1632_chipselect(0);
}

//**************************************************************************************************
//Function Name: ht1632_senddata
//Function Feature: send a nibble (4 bits) of data to a particular memory location of the
//                  ht1632.  The command has 3 bit ID, 7 bits of address, and 4 bits of data.
//                  Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 Free
//                  Note that the address is sent MSB first, while the data is sent LSB first!
//                  This means that somewhere a bit reversal will have to be done to get
//                  zero-based addressing of words and dots within words.
//Input Argument: chipNo: the chip you want to send data
//               address: chip address to write
//                  data: data to write to chip memory
//Output Argument: void
//**************************************************************************************************
static void ht1632_senddata (byte chipNo, byte address, byte data)
{
  ht1632_chipselect(chipNo);
  ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6); // Send address
  ht1632_writebits(data, 1<<3); // send 4 bits of data
  ht1632_chipselect(0);
}

//**************************************************************************************************
//Function Name: ht1632_clear
//Function Feature: clear display
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
void ht1632_clear()
{
  char i;

  for (int j=1;j<=CHIP_MAX;j++) {
    ht1632_chipselect(j);
    ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
    ht1632_writebits(0, 1<<6); // Send address
    for (i=0; i<64/2; i++) // Clear entire display
      ht1632_writebits(0, 1<<7); // send 8 bits of data
    ht1632_chipselect(0);
    for (i=0; i<64; i++)
      ht1632_shadowram[i][j] = 0;
  }
}

//**************************************************************************************************
//Function Name: xyToIndex
//Function Feature: get the value of x,y
//Input Argument: x: X coordinate
//                y: Y coordinate
//Output Argument: address of xy
//**************************************************************************************************
byte xyToIndex(byte x, byte y)
{
  byte nChip, addr;

  if (x>=32) {
    nChip = 3 + x/16 + (y>7?2:0);
  } 
  else {
    nChip = 1 + x/16 + (y>7?2:0);
  }

  x = x % 16;
  y = y % 8;
  addr = (x<<1) + (y>>2);

  return addr;
}

//**************************************************************************************************
//Function Name: calcBit
//Function Feature: calculate the bitval of y
//Input Argument: y: Y coordinate
//Output Argument: bitval
//**************************************************************************************************
#define calcBit(y) (8>>(y&3))

//**************************************************************************************************
//Function Name: get_pixel
//Function Feature: get the value of x,y
//Input Argument: x: X coordinate
//                y: Y coordinate
//Output Argument: color setted on x,y coordinates
//**************************************************************************************************
int get_pixel(byte x, byte y) {
  byte addr, bitval, nChip;

  if (x>=32) {
    nChip = 3 + x/16 + (y>7?2:0);
  } 
  else {
    nChip = 1 + x/16 + (y>7?2:0);
  }

  addr = xyToIndex(x,y);
  bitval = calcBit(y);

  if ((ht1632_shadowram[addr][nChip-1] & bitval) && (ht1632_shadowram[addr+32][nChip-1] & bitval)) {
    return ORANGE;
  } 
  else if (ht1632_shadowram[addr][nChip-1] & bitval) {
    return GREEN;
  } 
  else if (ht1632_shadowram[addr+32][nChip-1] & bitval) {
    return RED;
  } 
  else {
    return 0;
  }
}

//**************************************************************************************************
//Function Name: ht1632_plot
//Function Feature: plot a dot on x,y
//Input Argument: x: X coordinate
//                y: Y coordinate
//            color: BLACK(clean), GREEN, RED, ORANGE
//Output Argument: void
//**************************************************************************************************
void ht1632_plot (byte x, byte y, byte color)
{
  byte nChip, addr, bitval;

  if (x<0 || x>X_MAX || y<0 || y>Y_MAX)
    return;

  if (color != BLACK && color != GREEN && color != RED && color != ORANGE)
    return;

  if (x>=32) {
    nChip = 3 + x/16 + (y>7?2:0);
  } 
  else {
    nChip = 1 + x/16 + (y>7?2:0);
  }

  addr = xyToIndex(x,y);
  bitval = calcBit(y);

  switch (color)
  {
  case BLACK:
    if (get_pixel(x,y) != BLACK) { // compare with memory to only set if pixel is other color
      // clear the bit in both planes;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  case GREEN:
    if (get_pixel(x,y) != GREEN) { // compare with memory to only set if pixel is other color
      // set the bit in the green plane and clear the bit in the red plane;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  case RED:
    if (get_pixel(x,y) != RED) { // compare with memory to only set if pixel is other color
      // clear the bit in green plane and set the bit in the red plane;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  case ORANGE:
    if (get_pixel(x,y) != ORANGE) { // compare with memory to only set if pixel is other color
      // set the bit in both the green and red planes;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  }
}

//**************************************************************************************************
//Function Name: set_buffer
//Function Feature: set buffer variable to char
//Input Argument: chr: char to set on buffer
//Output Argument: void
//**************************************************************************************************
void set_buffer(char chr){
  for(int i=0; i<sizeof(chl); i++){
    if(chl[i] == chr){
      int pos = i*8;
      for(int j=0;j<8;j++){
        memcpy_P(buffer[j], (PGM_P)pgm_read_word(&(CHL[j+pos])), 8);
      }
    }
  }
}

//**************************************************************************************************
//Function Name: null_buffer
//Function Feature: null entire buffer
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
void null_buffer(){
  for(int i=0;i<8;i++)
    for(int j=0; j<8;j++)
      buffer[i][j] = 0;
}

//**************************************************************************************************
//Function Name: ht1632_initialize
//Function Feature: initialize the display
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
static void ht1632_initialize()
{
  pinMode(ht1632_cs, OUTPUT);
  digitalWrite(ht1632_cs, HIGH);
  pinMode(ht1632_wrclk, OUTPUT);
  pinMode(ht1632_data, OUTPUT);
  pinMode(ht1632_clk, OUTPUT);
  for (int i=1; i<=CHIP_MAX; i++) {
    ht1632_sendcmd(i, HT1632_CMD_SYSDIS); // Disable system
    ht1632_sendcmd(i, HT1632_CMD_COMS00); // 16*32, PMOS drivers
    ht1632_sendcmd(i, HT1632_CMD_MSTMD);  // MASTER MODE
    ht1632_sendcmd(i, HT1632_CMD_SYSON);  // System on
    ht1632_sendcmd(i, HT1632_CMD_LEDON);  // LEDs on 
  }
  ht1632_clear(); // Clear the display
}

void ht1632_draw_strings( char* STRING1, char* STRING2 )
{
  int x,i,j;
  for(x=1;x<5;x++) {
    null_buffer();
    for(i=0;i<8;i++){
      for(j=0;j<8;j++){
        set_buffer(STRING1[x-1]);
        if (~buffer[i][j] & (1<<0)) {
          ht1632_plot(j+(8*(x-1))-1,i,GREEN);
        } 
        else {
          ht1632_plot(j+(8*(x-1))-1,i,BLACK);
        }
      }
    }
  }
  for(x=1;x<5;x++) {
    null_buffer();
    for(i=0;i<8;i++){
      for(j=0;j<8;j++){
        set_buffer(STRING2[x-1]);
        if (~buffer[i][j] & (1<<0)) {
          ht1632_plot(j+(8*(x-1))-1,i+8,GREEN);
        } 
        else {
          ht1632_plot(j+(8*(x-1))-1,i+8,BLACK);
        }
      }
    }
  }
}


