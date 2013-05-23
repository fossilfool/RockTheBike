//#include <SoftPWM.h>

/**** Split-rail Pedalometer
* Arduino code to run the sLEDgehammer
* ver. 1.9
* Written by:
* Thomas Spellman <thomas@thosmos.com>
* Jake <jake@spaz.org>
* Paul@rockthebike.com
*
* Notes: 
* 1.6 - moved version to the top, started protocol of commenting every change in file and in Git commit
* 1.7 - jake 6-21-2012 disable minusalert until minus rail is pedaled at least once (minusAlertEnable and startupMinusVoltage)
* 1.8 -- FF added annoying light sequence for when relay fails or customer bypasses protection circuitry.+
* 1.9 - TS - cleaned up a bit, added state constants, turn off lowest 2 levels when level 3 and above
* Australia - changed pin numbers to match specific Arbduino setup
*/

char versionStr[] = "AC Power Pedal Power Utility Box ver. 1.9";

/*

Check the system voltage. 
Use a double array to check desired behavior for current voltage.
Do the desired behavior until the next check.

Repeat. 

*/

const int STATE_OFF = 0;
const int STATE_BLINK = 1;
const int STATE_BLINKFAST = 3;
const int STATE_ON = 2;

const int pwm = 0;
const int onoff = 1;

// PIN VARIABLES
const int numPins = 7; // Number of active Arduino Pins for + rail team.
const int relayPin = 13;
const int voltPin = A0; // Voltage Sensor Input

int pin[numPins] = {2, 3, 4, 5, 6, 7, 8}; // specific to Australia unit

// LEVEL VARIABLES - there can be more levels than there are pins
const int numLevels = 8;

// voltages at which to turn on each level
float levelVolt[numLevels] = {24.0, 28, 32, 36, 40, 44, 48, 50};

// output pins for each output level
int levelPin[numLevels] = {2, 3, 4, 5, 6, 7, 8, 8}; //Mark, please help! 


const float voltcoeff = 13.25;  // larger numer interprets as lower voltage 
const float ampcoeff = 7.4; 


//MAXIMUM VOLTAGE TO GIVE LEDS

const float maxVoltLEDs = 21.0; //LED
const float maxVoltPlusRail = 50;  // 
const float dangerVoltPlusRail = 52.0;


//Hysteresis variables
const float plusRailComeBackInVoltage = 40;
int plusRailHysteresis=0;
int dangerVoltageState=0;

// vars to store temp values
int adcvalue = 0;

//Voltage related variables. 
float voltage = 0;

//Current related variables
float plusRailAmps=0;
float ACAmps=0; 
int plusRailAmpsRaw;
int ACAmpsRaw;

// on/off state of each level (for use in status output)
int state[numLevels];
int desiredState[numLevels];

const int AVG_CYCLES = 50; // average measured voltage over this many samples
const int DISPLAY_INTERVAL_MS = 500; // when auto-display is on, display every this many milli-seconds
const int BLINK_PERIOD = 600;
const int FAST_BLINK_PERIOD = 150;

int readCount = 0; // for determining how many sample cycle occur per display interval


// timing variables for blink. Mark, there are too many of these. 
int blinkState=0;
int fastBlinkState=0;
unsigned long time = 0;
unsigned long lastFastBlinkTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long timeRead = 0;
unsigned long timeDisplay = 0;

// current display level
int level = -1;

// var for looping through arrays
int i = 0;
int x = 0;
int y = 0;


void setup() {
  Serial.begin(57600);

  Serial.println(versionStr);
  
  pinMode(voltPin,INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin,LOW);
 
  // init LED pins
  for(i = 0; i < numPins; i++) {
    pinMode(pin[i],OUTPUT);
    if(pinType[i] == pwm)
      analogWrite(pin[i],0);
    else if(pinType[i] == onoff)
      digitalWrite(pin[i],0);
  }

   getVoltages();
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
          if (((time - lastBlinkTime) > BLINK_PERIOD) && blinkState==1){
               //                  Serial.println("I just turned pwm to 0.");
             //     pwmValue=0;
                  blinkState=0;
                       //   analogWrite(levelPin[i], pwmValue);
                  lastBlinkTime=time;
                } else if (((time - lastBlinkTime) > BLINK_PERIOD) && blinkState==0){
                 //   Serial.println("I just turned blinkstate to 1.");
                   blinkState=1; 
                   lastBlinkTime=time;
                }
                

          if (((time - lastFastBlinkTime) > FAST_BLINK_PERIOD) && fastBlinkState==1){
               //                  Serial.println("I just turned pwm to 0.");
             //     pwmValue=0;
                  fastBlinkState=0;
                       //   analogWrite(levelPin[i], pwmValue);
                  lastFastBlinkTime=time;
                } else if (((time - lastFastBlinkTime) > FAST_BLINK_PERIOD) && fastBlinkState==0){
                 //   Serial.println("I just turned blinkstate to 1.");
                    fastBlinkState=1; 
                   lastFastBlinkTime=time;
                }  

 
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

      for(i = 0; i < numLevels; i++) {
        if(voltage >= levelVolt[i]){
          senseLevel = i;
          desiredState[i]=STATE_ON;
        } else desiredState[i]=STATE_OFF;
      }

      // if voltage is below the lowest level, blink the lowest level
      if (voltage < levelVolt[0]){
        senseLevel=0;
        desiredState[0]=STATE_BLINK;
      } 
      
      // turn off first 2 levels if voltage is above 3rd level
      if(voltage > levelVolt[2]){
        desiredState[0] = STATE_OFF;
        desiredState[1] = STATE_OFF;
      }

     if (dangerVoltageState){
        for(i = 0; i < numLevels; i++) {
          desiredState[i] = STATE_BLINKFAST;
        }
     }    
      
      level=senseLevel;
      
      // if at the top level, blink it fast
      if (level == (numLevels-1)){
        desiredState[level-1] = STATE_BLINKFAST; //workaround
      }
      
   
    
    // Do the desired states. 
    // loop through each led and turn on/off or adjust PWM

     
                
    for(i = 0; i < numLevels; i++) {

     // if(pinType[i]==pwm) {
      
        if(desiredState[i]==STATE_ON){
          digitalWrite(levelPin[i], HIGH);
         
          state[i] = STATE_ON;}
        else if (desiredState[i]==STATE_OFF){
          digitalWrite(levelPin[i], LOW);
          state[i] = STATE_OFF;}
        else if (desiredState[i]==STATE_BLINK && blinkState==1){
         digitalWrite(levelPin[i], HIGH);
         state[i] = STATE_BLINK;}
        else if (desiredState[i]==STATE_BLINK && blinkState==0){
         digitalWrite(levelPin[i], LOW);
         state[i] = STATE_BLINK;}
        else if (desiredState[i]==STATE_BLINKFAST && fastBlinkState==1){
         digitalWrite(levelPin[i], HIGH);
         state[i] = STATE_BLINK;}
        else if (desiredState[i]==STATE_BLINKFAST && fastBlinkState==0){
         digitalWrite(levelPin[i], LOW);
         state[i] = STATE_BLINK; 
         
      }
    //} 
    } //end for
    
    //Now show the - Team how hard to pedal. 
    
  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    printDisplay();
    readCount = 0;
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
