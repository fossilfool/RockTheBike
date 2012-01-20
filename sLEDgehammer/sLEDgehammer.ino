//#include <SoftPWM.h>

/**** sLEDgeHammer 
 * Arduino code to  run the sLEDgehammer
 * ver. 0.7
 * Written by:
 * Thomas Spellman <thomas@thosmos.com>
 * Jake <jake@spaz.org>
 */

// total number of levels
const int numLevels = 8;
// voltage spread to ramp over at each level
//const float rampVolts = 2.0;

// Arduino pin for each output level
int pin[numLevels] = {3, 5, 6, 9, 10, 11, 4, 7};

// voltages at which to turn on each level
float levelVolt[numLevels] = {10.0, 13.0, 16.0, 18.0, 21.0, 24.0, 27.0, 30.0};

const int pwm = 0;
const int onoff = 1;
// type of each level (pwm or onoff)
int levelType[numLevels] = {pwm, pwm, pwm, pwm, pwm, pwm, onoff, onoff};


// Arduino pin used as input to voltage sensor
const int voltpin = A0;  // Voltage Sensor Input


//adc = v * 10/110/5 * 1024 == v * 18.618181818181818;  // for a 10k and 100k voltage divider into a 10bit (1024) ADC that reads from 0 to 5V 
const float voltcoeff = 18.618181818181818;  

//MAXIMUM VOLTAGE TO GIVE LEDS
const float maxvolt = 12.0;  

// vars to store temp values
int adcvalue = 0;
float voltage = 0;

// on/off state of each level (for use in status output)
int state[numLevels];

const int AVG_CYCLES = 50; // average measured voltage over this many samples
const int DISPLAY_INTERVAL_MS = 500; // when auto-display is on, display every this many milli-seconds
int readCount = 0; // for determining how many sample cycle occur per display interval

// vars for current PWM duty cycle
int pwmValue;
//int pwmvalue24V;
boolean updatePwm = false;

// timing variables
unsigned long time = 0;
unsigned long timeRead = 0;
unsigned long timeDisplay = 0;

// minimum time to keep each level on for
unsigned long onTime = 1000;

unsigned long levelTime = 0;

// current display level 
int level = -1;

// var for looping through arrays
int i = 0;


void setup()  { 
  Serial.begin(57600); 

// Initialize Software PWM
  //SoftPWMBegin();
  
  pinMode(voltpin,INPUT);
  
  // init LED pins
  for(i = 0; i < numLevels; i++) {
    pinMode(pin[i],OUTPUT);
    if(levelType[i] == pwm) 
      analogWrite(pin[i],0);
    else if(levelType[i] == onoff)
      digitalWrite(pin[i],0);
  }
} 

boolean levelLock = false;
int senseLevel = -1;

void loop()  { 
  
  time = millis();
  
  getvoltage();
  setpwmvalue();
  readCount++;
  
  
  if(updatePwm) {
    
    // if we're not in a level lock, then see if we need to change the level
    if(!levelLock) {
      // find the level that *should* light
      senseLevel = -1;
      for(i = 0; i < numLevels; i++) {
        if(voltage >= levelVolt[i])
          senseLevel = i;
      }
      
      // if the sensed level is greater than current level, adjust upward by one level only and lock
      if(senseLevel > level) {
        level++;
        levelLock = true;
        levelTime = time;
      }
      else 
        level = senseLevel;
    }
    
    
    // loop through each led and turn on/off or adjust PWM
    for(i = 0; i < numLevels; i++) {

//      // if we're in level lock, then just update the pwm value
//      if(levelLock) {
//        if(level >= i && levelType[i] == pwm) 
//          analogWrite(pin[i], pwmValue);
//        continue;
//      }

      if(level >= i) {
        if(levelType[i] == pwm) 
          analogWrite(pin[i], pwmValue);
        else if(levelType[i] == onoff)
          digitalWrite(pin[i], HIGH);
        state[i] = 1;
      }
      else {
        if(levelType[i] == pwm)
          analogWrite(pin[i], 0);
        else if(levelType[i] == onoff)
          digitalWrite(pin[i], LOW);
        state[i] = 0;
      }
      
    } // end for loop
    
    if(time - levelTime > onTime)
      levelLock = false;
      
  } /// end if(updatePwm)
  
  
//  
//  // loop through each led and turn on/off
//  for(i = 0; i < numLeds; i++) {
//    // test if led has already been turned on less than onTime ago, then just adjust for voltage change
//    if(time - ledTime[i] < onTime) { 
//        analogWrite(ledPin[i], pwmValue);
//    }
//    // otherwise, if voltage is higher than threshold, either ramp or turn it on
//    else if (voltage >= ledVolt[i]) {
//      if(voltage - ledVolt[i] < rampVolts)
//        analogWrite(ledPin[i], pwmValue * ((voltage - ledVolt[i]) / rampVolts) );
//      else {
//        ledState[i] = 1;
//        ledTime[i] = time;
//        analogWrite(ledPin[i], pwmValue);
//      }
//    } else {
//      ledState[i] = 0;
//      analogWrite(ledPin[i], 0);
//    }
//  }

  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    printDisplay();
    readCount = 0;
  }
  
  updatePwm = false;
}

void setpwmvalue()
{
  
  // The effective voltage of a square wave is 
  // Veff = Vin * sqrt(k) 
  // where k = t/T is the duty cycle 
  
  // If you want to make 12V from a 20V input you have to use 
  // k = (Veff/Vin)^2 = (12/20)^2 = 0.36 

  int newVal = 0;

  if (voltage <= maxvolt) {
    newVal = 255.0;
  } 
  else {
    newVal = maxvolt / voltage * 255.0;
  }
  
//  if(voltage <= 24) {
//    pwmvalue24V = 255.0;
//  }
//  else {
//    pwmvalue24V = sq(24 / voltage) * 255.0;
//  }

  if(newVal != pwmValue) {
    pwmValue = newVal;
    updatePwm = true;
  }
}

void getvoltage(){
  adcvalue = analogRead(voltpin);
  voltage = average(adc2volts(adcvalue), voltage); 
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
 * 30V * 10k/110k = 2.72V      // at ADC input, for a 55V max input range
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

return v * 18.618181818181818;
}

float adc2volts(float adc){
  // v = adc * 110/10 * 5 / 1024 == adc * 0.0537109375;
  return adc * 0.0537109375; // 55 / 1024 = 0.0537109375; 
}

void printDisplay(){
  Serial.print("volts: ");
  Serial.print(voltage);
  Serial.print(", pwm value: ");
  Serial.print(pwmValue);
  Serial.print(", Levels ");
  for(i = 0; i < numLevels; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(state[i]);
    Serial.print(", ");
  }
  Serial.print("readCount: ");
  Serial.print(readCount);
  Serial.println("");
}


