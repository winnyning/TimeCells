// Arduino embedded code for some sort of science, I dunno

// *** Trial Types ***
// 1 -> beads front
// 2 -> beads back
// 3 -> yarn front
// 4 -> yarn back

// *** Servos ***
// s0 -> study door
// s1 -> study spinner
// s2 -> delay door
// s3 -> test spinner
// s4 -> return door

// *** Task Phase ***
// 1 -> Set up the current trial.
// 2 -> Rat does a nose poke and runs on the treadmill.
// 3 -> Rat opens the gate.
// 4 -> Rat makes a choice.


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 150;
#define SERVOMAX 600;

// How long does the rat need to hold nose-poke?
// 40000;; ryan
//babies : 4000
long BEAMTIME = 100000;
// For how long does the rat get water?
long FEEDTIME = 200;

// define the trial types and objects

int trialobj[]    = {0, 0, 1, 1};
int trialpos[]    = {0, 1, 0, 1};
int servopos[]    = {150, 600}; // test spinner
int servopos1[]   = {600, 300}; // study spinner
// match IR beams with trial types
int studypins[]   = {26, 26, 25, 25}; //study beads study yarn
int testcpins[]   = {32, 32, 33, 33}; //test beads test ayarn
int testicpins[]  = {33, 33, 32, 32}; //test yarn test beads
int studyfeed[]   = {50, 50, 51, 51}; //study beads study yarn
int testfeed[]    = {53, 53, 52, 52}; //test beads test yarn
int testbeep[]    = {9, 9, 9, 9}; //test yarn test beads

// general variables
int taskphase     = 1;
long breaktime     = 0;
int beamstat      = 0;
int serialdata    = 0;
long feedir        = 0;
long beepir        = 0;
long holdtime      = 0;
int trialtype     = 0;
// set up function, run once during Arduino setup
void setup ()
{
  // attach the servo controller
  Serial.begin(9600); // baud rate = 9600
  pwm.begin();
  pwm.setPWMFreq(60);

  // door to study
  pwm.setPWM(0, 0, 150);
  delay(50);
  // study spinner
  pwm.setPWM(1, 0, 150);
  delay(50);
  // delay door
  pwm.setPWM(2, 0, 150);
  delay(50);
  // test spinner
  pwm.setPWM(3, 0, 150);
  delay(50);
  // return arm access
  pwm.setPWM(4, 0, 150);
  delay(50);
  // hang on




  // load all IR readers
  // set up pins in the input-pullup mode
  pinMode(26, INPUT_PULLUP); //study beads
  pinMode(25, INPUT_PULLUP); //study yarn
  pinMode(27, INPUT_PULLUP); //treadmill
  pinMode(32, INPUT_PULLUP); //test beads
  pinMode(33, INPUT_PULLUP); //test yarn

  // set up output pins
  // speaker is pin 22
  pinMode(50, OUTPUT); // study beads water
  pinMode(51, OUTPUT); // study yarn water
  pinMode(53, OUTPUT); // test beads water
  pinMode(52, OUTPUT); // test yarn water
  pinMode(13, OUTPUT); // feeding indicator???
  pinMode(12, OUTPUT);

  // initialize output pins to defaults
  digitalWrite(50, 0);
  delay(10);
  digitalWrite(50, 1);
  delay(10);
  digitalWrite(51, 0);
  delay(10);
  digitalWrite(51, 1);
  delay(10);
  digitalWrite(53, 0);
  delay(10);
  digitalWrite(53, 1);
  delay(10);
  digitalWrite(52, 0);
  delay(10);
  digitalWrite(52, 1);

  digitalWrite(13, 0);
  digitalWrite(12, 1);

  // chirp
  tone(testbeep[0], 261);
  delay(500);
  noTone(testbeep[0]);


} // setup()

void loop ()
{

  if (taskphase == 1)
  {

    // Are there any data waiting for us?
    ////////////////////////////////////////////
    ///// SET UP THE TRIAL ////////////////////
    ////////////////////////////////////////////
    if (Serial.available() > 0)
    {

      // There are data waiting for us.
      // read the input as an integer
      serialdata = Serial.parseInt() - 1;
      Serial.print(serialdata+1);
      trialtype = serialdata;
      // expect the read data to be the trial type
     
      // position the study  spinner
      pwm.setPWM(1, 0, servopos1[trialobj[serialdata]]);
      delay(100);

      // open the study door
      pwm.setPWM(0, 0, 600);

      // advance task phase
      taskphase = 2;

      tone(9, 100);
      delay(100);
      noTone(9);
      return;
    }
    else
    {
      // NOTE: no data available, return to beginning
      return;
    }
  }
/////////////////////////////////////////
///// study phase ///////////////////////
///////////////////////////////////
  
  // this is the phase where were waiting for him to study
  else if (taskphase == 2)
  {
    // wait for the rat to do a nose-poke
    // when beamstat == 1, the beam is broken
    // serialdata -> trial type
    beamstat = !digitalRead(studypins[serialdata]);
    
    // the rat must hold for BEAMTIME (in 1/Bd)
    if (beamstat == 1)
    {
      holdtime++;
    }
    else
    {
      holdtime = 0;
    }

    if (holdtime > BEAMTIME)
    {
      // signal that rat did nose-poke
      Serial.print(1);
      // give water
      digitalWrite(studyfeed[serialdata], 0);
      delay(FEEDTIME*.5);
      digitalWrite(studyfeed[serialdata], 1);


      // we know hes out of the study and return doors way
      pwm.setPWM(0, 0, 150);
      delay(50);
      pwm.setPWM(4, 0, 150);
      delay(50);
      // set up the test spinner
      pwm.setPWM(3, 0, servopos[trialpos[serialdata]]);

      // reset general veriables
      beamstat  = 0;
      holdtime  = 0;

      // advance task phase
      taskphase = 3;

      // signal MATLAB that the rat did a nose poke

      tone(9, 261);
      delay(100);
      noTone(9);
      
    }
    else
    {
      return;
    }
  }
  ////////////////////////////////////////////
  // the treadmill phase //////////////////////
  //////////////////////////////////////////
  else if (taskphase == 3)
  {
    // listen to treadmill beam
    beamstat  = !digitalRead(27);


    // if beam is broken, send treadmill response
    if (beamstat == 1)
    {
      // signal MATLAB that the rat did a nose poke
      Serial.print(2);

      // while treadmill is running, take a break
      //      breaktime = 0;
      // indicate this is delay time
      digitalWrite(11, 0);

      // advance task phase
      taskphase = 4;
      
    // ******* THIS IS THE TREADMILL DELAY **********
    // 1000 is one second
      delay(1000);
      tone(9, 261);
      delay(100);
      noTone(9);
      // just testing
      pwm.setPWM(2, 0, 250);
      return;
    }
    else
    {
      // NOTE: waiting for rat
      return;
    }
  }
  ////////////////////////////////////////////
  // where we wait for him to respond
  /////////////////////////////////////////////
  else if (taskphase == 4)
  {
    // listen to beam near reward
    feedir    = feedir + !digitalRead(testcpins[trialtype]);
    beepir    = beepir + !digitalRead(testicpins[trialtype]);

    //    // close door in case of error
    //    if (feedir + beepir > 1 && feedir + beepir < 5)
    //    { // close door when either is ampled
          
    //    }

    // if one beam is broken for long enough
    if (feedir > BEAMTIME || beepir > BEAMTIME)
    {
      pwm.setPWM(2, 0, 150);
      if (feedir > BEAMTIME)
      {
        // give reward to rat
        digitalWrite(testfeed[trialtype], 0);
        digitalWrite(13, 1);
        delay(FEEDTIME);
        digitalWrite(testfeed[trialtype], 1);
        digitalWrite(13, 0);

        // flicker pin 13 (feeding indicator)

        // signal MATLAB that the rat was fed
        Serial.print(3);
        pwm.setPWM(2,0,150);
      }
      else
      {
        // rat made incorrect choice
        // cue the sad music
        pwm.setPWM(2,0,150);
        tone(9, 261);
        delay(500);
        noTone(9);

        // signal MATLAB that the rat wasn't fed
        Serial.print(4);
      }

      // open return door
      pwm.setPWM(4, 0, 600);

      // reset the task phase and indicators
      delay(500);
      taskphase     = 1;
      feedir        = 0;
      beepir        = 0;

      
    }
  } // end of taskphase 4
  
} // loop()
