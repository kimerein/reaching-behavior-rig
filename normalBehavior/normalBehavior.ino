
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

const long day = 86400000; // 86400000 milliseconds in a day
const long hour = 3600000; // 3600000 milliseconds in an hour
const long minute = 60000; // 60000 milliseconds in a minute
const long second = 1000; // 1000 milliseconds in a second
const uint8_t I2Caddress = 0x60; // use 0x60 as default address
const int chipSelect = 4;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(I2Caddress);

String serialStringOut(String outString, String appendString)
{
  Serial.print(outString);
  appendString += outString;
  return appendString;

}

String time()
{
  long timeNow = millis();
  String outString = "";
  int days = timeNow / day ; //number of days
  int hours = (timeNow % day) / hour; //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
  int minutes = ((timeNow % day) % hour) / minute ; //and so on...
  int seconds = (((timeNow % day) % hour) % minute) / second;

  // digital clock display of current time
  outString = serialStringOut(String(days, DEC), outString);
  outString = serialStringOut(printDigits(hours), outString);
  outString = serialStringOut(printDigits(minutes), outString);
  outString = serialStringOut(printDigits(seconds), outString);
  outString = serialStringOut("\r\n", outString);
  return outString;
}

String printDigits(byte digits)
{
  String outString = "";

  // utility function for digital clock display: prints colon and leading 0
  outString = serialStringOut(":", outString);
  if (digits < 10)
  {
    outString = serialStringOut("0", outString);
  }
  outString = serialStringOut(String(digits, DEC), outString);
  return outString;
}

class DataCard
{

    String dataFile;

  public:
    DataCard(int chpselect, String fileName)
    {
      Serial.print("Initializing SD card ...");
      // see if the card is present and can be initialized
      if (!SD.begin(chpselect))
      {
        Serial.println("Card failed or not present");
        return;
      }
      Serial.println("Card initialized");
      dataFile = fileName;
    }

    void Update(String dataString)
    {
      // open the file
      File data = SD.open(dataFile, FILE_WRITE);
      // if file is available, write to it
      if (data)
      {
        data.println(dataString);
        data.close();
        // print to serial port as well
        //      Serial.println(dataString);
      } else
      {
        Serial.println("Error open SD file");
        Serial.print(">");
        Serial.print(millis());
        Serial.print(">");
        time();
      }
    }
};

DataCard outSD(chipSelect, "output.txt");

class StepperDisk
{
    Adafruit_StepperMotor *myStepper;
    long stepSize; // distance for stepper motor to move per trial
    long updateDelay; // time between steps
    int stepsPerRevolution; // number of steps per revolution
    long motorRPM; // motor speed
    unsigned long lastUpdate; // last update of position
    int stepsSoFar = 0; // number of steps moved on this trial
    int writeCode;
    long delayBeforeTurn = 0; // in ms, delay before movement onset, triggered on trial start
    long trialOnset = 0;
    boolean accomplishedTurn = false;
    boolean goesForward = true;
    int countLoaderSteps = 0;
    long startOfLoad = 0; // time in ms when loader turn begins

  public:

    StepperDisk(int nSteps, int whichMotor, int eventLabel, boolean forwardDirection)
    {
      // Connect a stepper motor with nSteps per revolution in position whichMotor
      myStepper = AFMS.getStepper(nSteps, whichMotor);
      stepsPerRevolution = nSteps;
      writeCode = eventLabel;
      goesForward = forwardDirection;
    }

    void SetAngle(int nSteps)
    {
      if (goesForward == true)
      {
        myStepper->step(nSteps, FORWARD, MICROSTEP);
      } else
      {
        myStepper->step(nSteps, BACKWARD, MICROSTEP);
      }
    }

    void SetSpeed(long rpm)
    {
      myStepper->setSpeed(rpm); // in rotations per minute
      motorRPM = rpm;
    }


    void Update(long currStepSize, long waitToTurn, boolean trialStarting, boolean doMove)
    {
      long timeFromLoad; 
      if (doMove == false) // don't move wheel
      {
        return;
      }
      if (trialStarting == true) // start of a new trial
      {
        // set goal distance for motor to move on this trial
        stepSize = currStepSize;
        // at given rpm speed, calculate update delay
        updateDelay = (1 / stepsPerRevolution) * (60 / motorRPM); // in seconds
        stepsSoFar = 0; // reset to zero
        // set delay before motor starts turning
        delayBeforeTurn = waitToTurn;
        trialOnset = millis();
        accomplishedTurn = false;
      }
      if ((millis() - trialOnset) > delayBeforeTurn) // time to start turning
      {
        if ((millis() - lastUpdate) > updateDelay) // time to update
        {
          lastUpdate = millis();
          if (stepsSoFar < stepSize)
          {
            if (stepsSoFar == 0) // starting turn
            {
              String outString = "";
              outString = serialStringOut(String(writeCode), outString);
              outString = serialStringOut(">S>", outString);
              outString = serialStringOut(String(millis()), outString);
              outString = serialStringOut("\r\n", outString);
              outSD.Update(outString);
            }
            if (goesForward == true)
            {
              myStepper->step(1, FORWARD, MICROSTEP);
            } else
            {
              myStepper->step(1, BACKWARD, MICROSTEP);
            }
            stepsSoFar += 1; // moved one more step
          } else if ((stepsSoFar >= stepSize) && (accomplishedTurn == false)) // ending presenting wheel turn or finishing loader turn
          {
            if (currStepSize == 1) // this is the loader wheel
            {
              if (countLoaderSteps<6) // haven't yet finished load
              {
                timeFromLoad = millis() - startOfLoad;
                if (timeFromLoad > 500) // 500 ms delay between loader steps
                { 
                  if (goesForward == true)
                  {
                    myStepper->step(1, FORWARD, INTERLEAVE);
                  } else
                  {
                    myStepper->step(1, BACKWARD, INTERLEAVE);
                  }
                  countLoaderSteps = countLoaderSteps+1;
                  startOfLoad=millis();
                  if (countLoaderSteps == 6)
                  {
                    countLoaderSteps = 0; // reset to 0
                    accomplishedTurn = true; // finished loader turn
                  }
                }
              }
            } 
          }
        }
      }
    }
};

class FlashOnce
{

    int ledPin; // analog output pin number
    long OnTime; // milliseconds of on-time
    long OffTime; // milliseconds of off-time
    int ledState; // led state
    unsigned long previousMillis; // stores last time LED was updated
    boolean didOnce = false; // whether already flashed once
    int writeCode;
    long currDelay; // delay for this trial between wheel finishing turning and cue turning on

  public:
    FlashOnce(int pin, long on, long off, int eventLabel)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);
      OnTime = on;
      OffTime = off;
      ledState = LOW; // start off
      digitalWrite(ledPin, ledState); // Update the actual LED
      previousMillis = 0;
      writeCode = eventLabel;
    }

    void Update(boolean trialStarting, boolean turnCueOn)
    {
      unsigned long currentMillis = millis();
      if (turnCueOn == false) // don't turn cue on
      {
        return;
      }
      if (trialStarting == true)
      {
        // re-initialize so can turn on again
        didOnce = false;
        // check that LED is off
        ledState = LOW;
        digitalWrite(ledPin, ledState); // Update the actual LED
        previousMillis = currentMillis;
      }
      if ((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = LOW; // Turn it off
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">E>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        //      outString = serialStringOut(">", outString);
        //      outString = serialStringOut(time(), outString);
        outSD.Update(outString);
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime) && (didOnce == false)) // Haven't yet turned LED on
      {
        ledState = HIGH; // Turn it on
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">S>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        //      outString = serialStringOut(">", outString);
        //      outString = serialStringOut(time(), outString);
        outSD.Update(outString);
        didOnce = true; // Save that turned on once
      }
    }
};

class FlashRandom
{

    int ledPin; // analog output pin number
    long OnTime; // milliseconds of on-time
    long OffTime; // milliseconds of off-time
    int ledState; // led state
    int minDelay; // minimum time delay between random LED transitions, in milliseconds
    int maxDelay; // maximum time delay between random LED transitions, in milliseconds
    unsigned long previousMillis; // stores last time LED was updated
    int writeCode;
    long delayToCue; // delay from start of trial to cue in milliseconds
    boolean haveReset = false; // whether have reset distractor at end of wheel turning
    int distractP; // probability that distractor LED is on versus off
    long timeAtTurn;
    long cueOn;
    long newrand;

  public:
    FlashRandom(int pin, int unconnectedPin, int minRand, int maxRand, int eventLabel, long off, int distractorP, long cueDuration)
    {
      ledPin = pin;
      pinMode(ledPin, OUTPUT);
      minDelay = minRand;
      maxDelay = maxRand;
      // OnTime = random(minRand, maxRand);
      OnTime = cueDuration;
      OffTime = random(minRand, maxRand);
      ledState = HIGH;
      digitalWrite(ledPin, ledState); // Update the LED
      previousMillis = 0;
      writeCode = eventLabel;
      delayToCue = off;
      distractP = distractorP;
      cueOn = cueDuration;
    }

    void Update(boolean trialStarting)
    {
      unsigned long currentMillis = millis();
      if (trialStarting == true)
      {
        // turn off LED
        ledState = HIGH; 
        digitalWrite(ledPin, ledState);
        timeAtTurn = currentMillis;
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">E>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
        haveReset = false;
      }
      else if (currentMillis - timeAtTurn < delayToCue) // pellet wheel is turning
      {
        // leave LED off
      }
      else if ((currentMillis - timeAtTurn >= delayToCue) && (haveReset == false))
      {
        // use distractP to determine probability that LED turns back on
        newrand = random(0, 100);
        if (newrand < distractP)
        {
          // turn LED on
          ledState = LOW;
          digitalWrite(ledPin, ledState); // Update the actual LED
          String outString = "";
          outString = serialStringOut(String(writeCode), outString);
          outString = serialStringOut(">S>", outString);
          outString = serialStringOut(String(millis()), outString);
          outString = serialStringOut("\r\n", outString);
          outSD.Update(outString);
          previousMillis = currentMillis; // Remember the time
          OnTime = cueOn; // in ms
          OffTime = random(minDelay, maxDelay); // in ms
        }
        haveReset = true;
      }
      else if ((ledState == LOW) && (currentMillis - previousMillis >= OnTime))
      {
        ledState = HIGH; // Turn it off
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">E>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
        OnTime = cueOn; // in ms
        OffTime = random(minDelay, maxDelay); // in ms
      }
      else if ((ledState == HIGH) && (currentMillis - previousMillis >= OffTime))
      {
        ledState = LOW; // Turn it on
        previousMillis = currentMillis; // Remember the time
        digitalWrite(ledPin, ledState); // Update the actual LED
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">S>", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
      }
    }
};

class AnalogSensor
{

    int sensorPin;
    int writeCode;
    int sensorReading;
    unsigned long previousMillis; // stores last time sensor value was written to SD card
    unsigned long updatePeriod; // write sensor value to SD card every updatePeriod ms

  public:
    AnalogSensor(int pin, int eventLabel, unsigned long writePeriod)
    {
      sensorPin = pin;
      writeCode = eventLabel;
      updatePeriod = writePeriod;
    }

    void Update()
    {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis > updatePeriod) 
      {
        // write sensor value
        String outString = "";
        outString = serialStringOut(String(writeCode), outString);
        outString = serialStringOut(">", outString);
        outString = serialStringOut(String(GetReading()), outString);
        outString = serialStringOut(">", outString);
        outString = serialStringOut(String(millis()), outString);
        outString = serialStringOut("\r\n", outString);
        outSD.Update(outString);
        previousMillis = currentMillis; // Remember the time
      }
    }

    int GetReading()
    {
      sensorReading = analogRead(sensorPin);
      return sensorReading;
    }
};

class ThresholdSensor
{

    long threshold = 1;
    int sensorPin;
    int count = 0;
    int writeCode;

  public:
    ThresholdSensor(int pin, long thresh, int eventLabel)
    {
      threshold = thresh;
      sensorPin = pin;
      writeCode = eventLabel;
    }

    void Update()
    {
      long sensorReading = analogRead(sensorPin);
      // if sensor reading is greater than threshold
      if (sensorReading >= threshold)
      {
        count += 1; // increment count
      }
    }

    int GetCount()
    {
      return count;
    }
};

class DigTrigger
{

    int camPin;
    unsigned long trigDur;
    int trigState; // trigger state
    unsigned long startAcquiringDelay; // delay after trial start to begin camera acquisition
    unsigned long acquireUntil; // acquire until this many seconds after trial start 
    long offTime;
    unsigned long trialStartMillis; // when the trial started in ms
    unsigned long previousMillis; // stores last time triggered

  public:
    DigTrigger(int cPin, unsigned long triggerDur, long trigOffTime, unsigned long startAcq, unsigned long acqUntil)
    {
      camPin = cPin;
      trigDur = triggerDur;
      offTime = trigOffTime; 
      startAcquiringDelay = startAcq;
      acquireUntil = acqUntil;
      pinMode(camPin, OUTPUT);
      trigState = LOW; // start off
      digitalWrite(camPin, trigState); // Update the trigger pin
      previousMillis = 0;
    }

    void Update(boolean trialStarting, boolean turnCamOn)
    {
      unsigned long currentMillis = millis();
      if ((trigState == HIGH) && (currentMillis - previousMillis >= trigDur)) // only want trigger on for trigDur duration
      {
        trigState = LOW;
        digitalWrite(camPin, trigState);
        previousMillis = currentMillis;
      } else if ((trialStarting == true) && (turnCamOn == true)) // beginning of a trial
      {
        previousMillis = currentMillis;
        trialStartMillis = currentMillis;
      } else if ((trigState == LOW) && (currentMillis - trialStartMillis > startAcquiringDelay) && (currentMillis - previousMillis >= offTime) && (currentMillis - trialStartMillis <= acquireUntil)) // turn on camera acquisition
      {
        trigState = HIGH;
        digitalWrite(camPin, trigState);
        previousMillis = currentMillis;
      }
    }
};

void reseedRandom( void )
{
  static const uint32_t HappyPrime = 937;
  union
  {
    uint32_t i;
    uint8_t b[4];
  }
  raw;
  int8_t i;
  unsigned int seed;
  
  for ( i=0; i < sizeof(raw.b); ++i )
  {
    raw.b[i] = EEPROM.read( i );
  }

  do
  {
    raw.i += HappyPrime;
    seed = raw.i & 0x7FFFFFFF;
  }
  while ( (seed < 1) || (seed > 2147483646) );

  Serial.println(seed);
  randomSeed( seed );  

  for ( i=0; i < sizeof(raw.b); ++i )
  {
    EEPROM.write( i, raw.b[i] );
  }
};

// Declare constants
const int loaderN = 1;
const int pelletsN = 2;
const uint8_t motorShieldAddress = 0x60;
const int stepsForStepper = 200;
const int cuePin = 3; // digital output pin that controls cue
const int optoPin = 6; // digital output pin that controls opto for silencing 
const int cameraPin = 5; // digital output pin that sends trigger to cameras
const long cameraTriggerDur = 2; // ms duration of camera digital output trigger
const long cameraTriggerOffTime = 2; // 2000; //2; // ms duration of time between camera triggers
const long startCamAcq = 500; // ms after trial onset to begin camera data acquisition
const long endCamAcq = 9000; // ms after trial onset to end camera data acquisition
const long cueDuration = 250; // cue on duration in ms
const long cueDelay = 1500; // delay from trial start until cue turns on in ms
const long optoDuration = 1000; // opto on duration in ms
const long optoDelay = 1495; // delay from trial start until opto turns on in ms
const int distractorPin = 2; // digital output pin that controls distractor
const int emptyPin = 4; // an analog input pin that is not connected (for random seed initialization)
const int randomMin = 1000; // in ms
const int randomMax = 10000; // in ms
const int probabilityDistractor = 2; // probability that distractor will be on versus off
const int beginTrialWrite = 0;
const int loaderWriteCode = 1;
const int pelletsWriteCode = 2;
const int interlockWriteCode = 6;
const int cueWriteCode = 4;
const int distractorWriteCode = 5;
const int optoWriteCode = 3;
const long minITI = 9050;
const long maxITI = 13000;
const long pelletsDelay = 0; // in ms
const long loaderDelay = 3250; // in ms
const unsigned long interlockUpdate = 5000; // in ms, how often to write interlock value
const int maxConsecutiveLoad = 4; // max number of consecutive trials for pellet loading, to prevent cameras maxing out
const int probOfOpto = 0; // turn on opto in this percent of trials
const int fractionPelletsNotLoaded = 60; // fraction of wheel turns in which pellet is NOT presented

long stepperMotorRPM = 100; // rpm
long loaderMotorRPM = 20; // rpm
boolean trialIsStarting = false;
int trialState = LOW;
long ITI = 0;
unsigned long prevMs = 0;
unsigned long currMs = 0;
int nTrials = 0;
int isLoaded[6]={0,0,0,0,0,0}; // whether pellet is loaded in each hole of pellet presenter wheel
int trialCount = 0; // count how many trials since last cue 
int nTrialsBetweenCues = 0; // whether to turn wheel 2 or 3 times between cues
boolean doLoad = false; // whether to load pellet this trial
boolean pelletToMouse = false; // if pellet is available to mouse on this trial
int optoThisTime = 0;
int consecLoaded = 0;

// probabilityDistractor for FlashRandom should match ratio of cueDuration to expected value of [randomMin, randomMax]
FlashRandom distractor(distractorPin, emptyPin, randomMin, randomMax, distractorWriteCode, cueDelay + cueDuration, probabilityDistractor, cueDuration);
FlashOnce cue(cuePin, cueDuration, cueDelay, cueWriteCode);
FlashOnce opto(optoPin, optoDuration, optoDelay, optoWriteCode);
StepperDisk loader(stepsForStepper, loaderN, loaderWriteCode, true);
StepperDisk pellets(stepsForStepper, pelletsN, pelletsWriteCode, true);
AnalogSensor interlock(A0, interlockWriteCode, interlockUpdate);
DigTrigger cameras(cameraPin, cameraTriggerDur, cameraTriggerOffTime, startCamAcq, endCamAcq);

void setup()
{
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.println("Program Begins");
  AFMS.begin();
  loader.SetSpeed(loaderMotorRPM);
  pellets.SetSpeed(stepperMotorRPM);
  reseedRandom();
}

void loop()
{
  currMs = millis();
  // if ITI has elapsed, initiate new trial
  if ((trialState == LOW) && ((currMs - prevMs) > ITI))
  {
    prevMs = currMs;
    trialState = HIGH;
    ITI = random(minITI, maxITI);
    // write ITI
    String outputString = "";
    outputString = serialStringOut(String(beginTrialWrite), outputString);
    outputString = serialStringOut(">", outputString);
    outputString = serialStringOut(String(ITI), outputString);
    outputString = serialStringOut(">", outputString);
    outputString = serialStringOut(String(millis()), outputString);
    outputString = serialStringOut("\r\n", outputString);
    outSD.Update(outputString);
    // increment trial count
    nTrials += 1;
    // 2 or 3 trials between cues?
    if (trialCount >= nTrialsBetweenCues)
    {
      // choose next trial interval between cues
      int randval = random(1, 100);
      if ((consecLoaded < maxConsecutiveLoad-1) && (randval > fractionPelletsNotLoaded)) {
        nTrialsBetweenCues = 0; // sets next trial as pellet loading trial
      } else {
        nTrialsBetweenCues = 1; // skip pellet loading on next trial
      }
      trialCount = 0;
      // this is a load trial
      doLoad = true;
      consecLoaded = consecLoaded + 1;
    } else {
      // increment count of trials since cue
      trialCount = trialCount + 1;
      // this is a trial where no pellet is loaded
      doLoad = false;
      consecLoaded = 0;
    }
    // update pellet buffer
    int i;
    for (i = 5; i >= 1; i = i - 1) {
      isLoaded[i] = isLoaded[i-1];
    }
    if (doLoad == true) {
      isLoaded[0] = 1; // 1 if pellet is loaded this trial
    } else {
      isLoaded[0] = 0; // 0 if pellet is not loaded this trial
    }
    // read off whether pellet was loaded 5 turns ago -- if yes, then pellet is now in front of mouse
    if (isLoaded[5] == 1) {
      // pellet is available to mouse
      pelletToMouse = true;
      Serial.println(isLoaded[5]);
    } else {
      pelletToMouse = false;
      Serial.println(isLoaded[5]);
    }
  } else if (trialState == HIGH)
  {
    trialState = LOW;
  }

  if (trialState == LOW)
  {
    trialIsStarting = false;
  } else if (trialState == HIGH)
  {
    trialIsStarting = true;
    int randnum = random(0, 100);
    if (randnum <= probOfOpto)
    {
      optoThisTime = 1;
    } else {
      optoThisTime = 0;
    }
  }
  cameras.Update(trialIsStarting, pelletToMouse); // want to trigger camera on when there's a pellet being presented (i.e., real trial), trigger off after as long as possible
  loader.Update(1, loaderDelay, trialIsStarting, doLoad);
  cameras.Update(trialIsStarting, pelletToMouse); 
  pellets.Update(20, pelletsDelay, trialIsStarting, true); 
  cameras.Update(trialIsStarting, pelletToMouse); 
  opto.Update(trialIsStarting, (pelletToMouse && optoThisTime)); // only turn on opto in probOfOpto trials 
  cameras.Update(trialIsStarting, pelletToMouse); 
  cue.Update(trialIsStarting, pelletToMouse);
  cameras.Update(trialIsStarting, pelletToMouse); 
  distractor.Update(trialIsStarting);
  interlock.Update();
  cameras.Update(trialIsStarting, pelletToMouse); 
}
