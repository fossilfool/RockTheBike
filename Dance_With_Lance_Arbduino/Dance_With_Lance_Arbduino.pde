//#include <SoftPWM.h>

/**** Split-rail Pedalometer
* Arduino code to run the sLEDgehammer
* ver. 1.6
* Written by:
* Thomas Spellman <thomas@thosmos.com>
* Jake <jake@spaz.org>
* Paul@rockthebike.com
*
* Notes: 
* 1.6 - moved version to the top, started protocol of commenting every change in file and in Git commit
* 1.7 - jake 6-21-2012 disable minusalert until minus rail is pedaled at least once (minusAlertEnable and startupMinusVoltage)
* 1.8 -- FF added annoying light sequence for when relay fails or customer bypasses protection circuitry.+
* Australia - changed pin numbers to match specific Arbduino setup
*/

char versionStr[] = "AC Power Pedal Power Utility Box ver. 1.8";

/*

Check the system voltage. 
Use a double array to check desired behavior for current voltage.
Do the desired behavior until the next check.

Repeat. 

*/


const int pwm = 0;
const int onoff = 1;

const int numLevels = 8;

// Arduino pin for each output level
const int numPins = 7; // Number of active Arduino Pins for + rail team.
int pin[numPins] = {2,3,4, 5, 6,7,8}; // specific to Australia unit
const int relayPin=13;  // specific to Australia unit

// voltages at which to turn on each level
//float levelVolt[numLevels] = {21, 24.0, 27.0};
float levelVolt[numLevels] = {24.0, 28, 32, 36, 40, 44, 48, 50};
//float levelVolt[numLevels] = {15.0, 17, 19.5, 22.2, 23};
int levelMode=0; // 0 = off, 1 = blink, 2 = steady
int whichPin[]={2,3,4,5,6,7,8,8}; //Mark, please help! 

// voltages at which to turn on each level
//float levelVolt[numLevels] = {21, 24.0, 27.0};
//float minusRailLevelVolt[numLevels] = {22.0, 23.5, 24.8, 26.7, 27.2};
//int minusRailLevelMode=0; // 0 = off, 1 = blink, 2 = steady

int levelType[numLevels] = {pwm, pwm, pwm, pwm, pwm, pwm, pwm, pwm};

wh33l;'wh33l;'wh33l;'
// type of each level (pwm or onoff)



// Arduino pins to be used as inputs to voltage sensor. This shows where to connect the wires! 
const int voltPin = A0; // Voltage Sensor Input
//const int minusVoltPin = A1; // Voltage Sensor Input
//const int ACAmpsPin = A2; // Voltage Sensor Input
//const int plusRailAmpsPin = A1; 

//const int twelveVoltPin=12;
//const int twelveVoltProtectionPin=8;

/*
Pin A0 --- AttoPilot "V"
Pin A1 --- AttoPilot "I"
GND ------ AttoPilot "GND"
GND ------ SerLCD "GND"
5V ------- SerLCD "VCC"
*/

//const int minusRelayPin=12;
//adc = v * 10/110/5 * 1024 == v * 18.618181818181818; // for a 10k and 100k voltage divider into a 10bit (1024) ADC that reads from 0 to 5V

const float voltcoeff = 13.25;  // larger numer interprets as lower voltage 
const float ampcoeff = 7.4; 

//MAXIMUM VOLTAGE TO GIVE LEDS

const float maxVoltLEDs = 21.0; //LED
const float maxVoltPlusRail = 50;  // 
const float dangerVoltPlusRail = 52.0;


//Hysteresis variables
const float plusRailComeBackInVoltage = 40;
//const float plusRailComeBackInVoltagetwelveVoltMode = 13.7;
int plusRailHysteresis=0;
int dangerVoltageState=0;
//int minusRailHysteresis=0;

// vars to store temp values
int adcvalue = 0;

//Voltage related variables. 
float voltage = 0;
//float minusRailVoltage=0;
//float startupMinusVoltage=0;  // inital read of minus rail, to see if minus is being pedaled at all

//Current related variables
float plusRailAmps=0;
float ACAmps=0; 
int plusRailAmpsRaw;
int ACAmpsRaw;

// on/off state of each level (for use in status output)
int state[numLevels];
int desiredState[numPins];

const int AVG_CYCLES = 50; // average measured voltage over this many samples
const int DISPLAY_INTERVAL_MS = 500; // when auto-display is on, display every this many milli-seconds
const int VICTORYTIME=4000;

int readCount = 0; // for determining how many sample cycle occur per display interval

// vars for current PWM duty cycle
int pwmValue;
//int pwmvalue24V;
boolean updatePwm = false;
//boolean minusAlert = false;  // makes all the pedalometer lights flash because minus rail is low
//boolean minusAlertEnable = false;  // only enable the minusAlert if minus rail is pedaled at all
//const float minusAlertVoltage = 14.5;  // below this voltage, if enabled, minusAlert happens.
//



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

// Time related variables. Mark, this may be a vestige of the sLEDgehammer code but we should bring this feature back because in this video, the flickering would be prevented:
//Basically this is a minimum time to keep each level on for before you can change it. http://www.rockthebike.com/pedal-power-utility-box-from-rock-the-bike/  
unsigned long onTime = 500;
unsigned long levelTime = 0;

// current display level
int level = -1;

// var for looping through arrays
int i = 0;
int x = 0;
int y = 0;


void setup() {
  Serial.begin(57600);

// Initialize Software PWM
  //SoftPWMBegin();
  
  Serial.println(versionStr);
  
  pinMode(voltPin,INPUT);
//  pinMode(minusVoltPin, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin,LOW);
//  pinMode(minusRelayPin, OUTPUT);
//   pinMode(twelveVoltPin, INPUT);
//      pinMode(twelveVoltProtectionPin, INPUT);
 
  // init LED pins
//  pinMode(10, OUTPUT); // wtf?  this is relay pin -jake
  for(i = 0; i < numLevels; i++) {
    pinMode(pin[i],OUTPUT);
    if(levelType[i] == pwm)
      analogWrite(pin[i],0);
    else if(levelType[i] == onoff)
      digitalWrite(pin[i],0);
//       digitalWrite(twelveVoltPin, HIGH);
//       digitalWrite(twelveVoltProtectionPin, HIGH);
  }

//  twelveVoltPedalometerMode=false;
  // digitalWrite(minusRelayPin, LOW);
   getVoltages();
//   startupMinusVoltage = minusRailVoltage;  // we don't enable minusAlert unless minus rail gets pedaled up from the voltage it started at
}

boolean levelLock = false;
int senseLevel = -1;

void loop() {

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
                       //   analogWrite(whichPin[i], pwmValue);
                  lastBlinkTime=currentTime;
                } else if (((currentTime - lastBlinkTime) > 600) && blinkState==0){
                 //   Serial.println("I just turned blinkstate to 1.");
                   blinkState=1; 
                   lastBlinkTime=currentTime;
                }
                

          if (((currentTime - lastFastBlinkTime) > 180) && fastBlinkState==1){
               //                  Serial.println("I just turned pwm to 0.");
             //     pwmValue=0;
                  fastBlinkState=0;
                       //   analogWrite(whichPin[i], pwmValue);
                  lastFastBlinkTime=currentTime;
                } else if (((currentTime - lastFastBlinkTime) > 150) && fastBlinkState==0){
                 //   Serial.println("I just turned blinkstate to 1.");
                    fastBlinkState=1; 
                   lastFastBlinkTime=currentTime;
                }  


//Test for Twelve Volt Mode
// Serial.println ("Twelve Volt Mode is in effect."); 



//Serial.println( twelveVoltProtectionMode ? "Twelve Volt Protection Mode is true" : "Twelve Volt Protection Mode is false");
  
  //protect the ultracapacitors if needed


 
if (voltage > maxVoltPlusRail){

   digitalWrite(relayPin, HIGH); 
   plusRailHysteresis=1;
  }
  
if (plusRailHysteresis==1 && voltage < plusRailComeBackInVoltage){
 digitalWrite(relayPin, LOW);
 plusRailHysteresis=0;
 } 

if (voltage > dangerVoltPlusRail){
   dangerVoltageState=1;
  } else { 
    dangerVoltageState=0;
  } 
  
    //Set the desired lighting states. 
      
      senseLevel = -1;
      if (voltage <=levelVolt[0]){
        senseLevel=0;
    //    Serial.println("Level Mode = 1");
        desiredState[0]=1;
      } else {
      
      for(i = 0; i < numLevels; i++) {
        
        if(voltage >= levelVolt[i]){
          senseLevel = i;
          desiredState[i]=2;
          levelMode=2;
        } else desiredState[i]=0;
     //   if (minusAlert == true) desiredState[i] = 3;  // make all lights blink if minusAlert is true, get attention to text display
      }
      }
      
      level=senseLevel;
  //    Serial.print("Level: ");
   //   Serial.println(level);
     if (level == (numLevels-1)){
  // Serial.print("LevelMode = 1");
        desiredState[level-1] = 3; //workaround
      }
      
      // End setting desired states. 
   if (dangerVoltageState){
      for(i = 0; i < numLevels; i++) {
        desiredState[i] = 3;
      }
   }    
   
    
    // Do the desired states. 
    // loop through each led and turn on/off or adjust PWM

     
                
    for(i = 0; i < numPins; i++) {

      if(levelType[i]==pwm) {
      
        if(desiredState[i]==2){
          analogWrite(whichPin[i], pwmValue);
         
          state[i] = 2;}
        else if (desiredState[i]==0){
          analogWrite(whichPin[i], 0);
          state[i] = 0;}
        else if (desiredState[i]==1 && blinkState==1){
         analogWrite(whichPin[i], pwmValue);
         state[i] = 1;}
        else if (desiredState[i]==1 && blinkState==0){
         analogWrite(whichPin[i], 0);
         state[i] = 1;}
        else if (desiredState[i]==3 && fastBlinkState==1){
         analogWrite(whichPin[i], pwmValue);
         state[i] = 1;}
        else if (desiredState[i]==3 && fastBlinkState==0){
         analogWrite(whichPin[i], 0);
         state[i] = 1; 
         
      }
    } 
    } //end for
    
    //Now show the - Team how hard to pedal. 
    
   

    
  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    printDisplay();
    readCount = 0;
  }
  
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
 

 plusRailAmps = average(adc2amps(adcvalue), plusRailAmps);
 
}

void getVoltages(){
 
 //first two lines are for + rail
  adcvalue = analogRead(voltPin);
  voltage = average(adc2volts(adcvalue), voltage);
  
}

//Future simplification / generalization
void protectUltracap(int whichRail){

  // Turn OFF the relay to protect either the minus or the plus rail 
  
}

//Future simplification / generalization
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
  Serial.print("volts: ");
  Serial.print(voltage);
    Serial.print(", AC Amps: ");
  Serial.print(ACAmps);

  Serial.print(", Levels ");
  for(i = 0; i < numLevels; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(state[i]);
    Serial.print(", ");
  }
  
  Serial.println("");
  Serial.println();
  
}
