#include <MIDI.h>                         //header files
#include <TM1637Display.h>
#include <TimerOne.h>
#include <EEPROM.h>

// byte *para_focus = &g_bpm;             //parameter focus
//para_focus = &g_vel;                    //change parameter pointer  
//*para_focus < r_lowest                  //get pointer value                   

const int CLK = 10;                       //Set the CLK pin connection to the display
const int DIO = 11;                       //Set the DIO pin connection to the display
TM1637Display display(CLK, DIO);          //set up the 4-Digit Display.

//rotary encoder variables
static int pinA = 3;                      // Our first hardware interrupt pin is digital pin 2
static int pinB = 2;                      // Our second hardware interrupt pin is digital pin 3
static int e_pin = 13;                    // encoder switch pin
volatile byte aFlag = 0;                  // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0;                  // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile int encoderPos = 0;              //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile int encoderStep = 1;
volatile int oldEncPos = 0;               //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0;                //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte step_v[16];                          //step backup array
unsigned long previousMillis = 0;         //store last time
unsigned long bpmMillis = 0;              //store last time for bpm

MIDI_CREATE_DEFAULT_INSTANCE();

byte midi_start = 0xfa;                   //midi transport controls
byte midi_stop = 0xfc;                    //stop
byte midi_clock = 0xf8;                   //clock sync
byte midi_continue = 0xfb;                //continue
byte data;
byte midi_tap;                            //midi clock click 
byte step_tap;                            //sequencer step
byte step_cursor;                         //step edit cursor
int pot_data;                             //analogue pot data
byte b_data;                              //BPM level 
int c_data;                               //CC level
int ct_data;                              //temporary CC level
int s_data;                               //sync data: 0=internal 1=external
int shift_data;                           //beat shift 
int o_data;                               //offset level
int d_data;                               //display mode
byte m_data;                              //MIDI potentiometer
byte EEPROM_ADDRESS;                      //eeprom save location
int msend;                                //MIDI value send
int brightness;                           //led brightness

int bpm;                                  //internal BPM 
int mode;                                 //select mode setting
int m_chan;                               //MIDI channel

byte b_offset;                            //internal bpm offset REDUNDANT
byte para;                                //parameter value
byte d_state;                             //display state: 0=parameter (default) 1=BPM 2=CC 3=SYNC 4=SHIFT 5=OFFSET
byte p_state;                             //play state: 0=play (default) 1= hold 2=stop
byte p_butt = 0;                          //play button state
byte e_butt = 0;                          //edit button state

#define CLOCKS_PER_BEAT 24
#define PRINT_INTERVAL 10000
#define MINIMUM_TAPS 3  //                                  REDUNDANT
#define EXIT_MARGIN 150 // If no tap after 150% of last tap interval -> measure and set  REDUNDANT

#define DEBOUNCE_INTERVAL 500L // Milliseconds              REDUNDANT

long intervalMicroSeconds;
volatile int blinkCount = 0;     //                           REDUNDANT
#define BLINK_TIME 4 // How long to keep LED lit in CLOCK counts (so range is [0,24])    REDUNDANT

//  ___
// F|A|B
//  -G-
// E|D|C
//  ---

//const uint8_t SEG_DONE[] = {
//  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,    // A
//  SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,            // b
//  SEG_A | SEG_D | SEG_E | SEG_F,                    // c
//  SEG_G | SEG_D | SEG_E,                            // C
//  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,            // d
//  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,            // E
//  SEG_A | SEG_E | SEG_F | SEG_G,                    // F
//  SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,    // g
//  SEG_C | SEG_E | SEG_G ,                           // n
//  SEG_C | SEG_E | SEG_D ,                           // u
//  SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,            // y
//  SEG_D | SEG_E | SEG_F | SEG_G,                    // t
//  SEG_C | SEG_D | SEG_E | SEG_G,                    // o
//  SEG_C | SEG_E | SEG_G | SEG_F                     // h
//  };
//  
const uint8_t S_p[] = {
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,0x0,0x0,0x0             // P
};

const uint8_t S_pmin[] = {
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,SEG_G,0x0,0x0             // P-
};

const uint8_t S_omin[] = {
    SEG_C | SEG_D | SEG_E | SEG_G,            //o
    SEG_G,0x0,0x0                             // -
};

const uint8_t S_b[] = {
    SEG_C | SEG_D | SEG_E | SEG_F | SEG_G  ,0x0,0x0, 0x0          // b
};

const uint8_t S_S[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G ,0x0,0x0, 0x0           // S
};

const uint8_t S_Smin[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G , SEG_G ,0x0, 0x0       // S-
};

const uint8_t S_sync[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G,            // S
    SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,            // y
    SEG_C | SEG_E | SEG_G ,                           // n
    SEG_G | SEG_D | SEG_E                             // c
};

const uint8_t S_shft[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G,            // S
    SEG_C | SEG_E | SEG_G | SEG_F | SEG_B,            // H
    SEG_A | SEG_E | SEG_F | SEG_G,                    // F
    SEG_D | SEG_E | SEG_F | SEG_G                     // t
};

const uint8_t S_c[] = {
    SEG_A | SEG_D| SEG_E | SEG_F ,0x0,0x0,0x0         // C
};

const uint8_t S_c2[] = {
    SEG_D | SEG_E | SEG_G ,0x0,0x0,0x0                // c
};

const uint8_t S_plus[] = {
    0x0,
    SEG_G | SEG_B| SEG_C ,                            // plus
    SEG_G,
    0x0
};

const uint8_t S_minus[] = {
    0x0,
    SEG_G,                                            // minus
    SEG_G,
    0x0
};

const uint8_t S_no[] = {
    SEG_D | SEG_E | SEG_G,   
    SEG_C | SEG_E | SEG_G | SEG_F, 
    SEG_G,                                            // ch--
    SEG_G
};

const uint8_t S_ch[] = {
    SEG_D | SEG_E | SEG_G,   
    SEG_C | SEG_E | SEG_G | SEG_F    
    ,0x0,0x0                                          // ch
};

const uint8_t S_off[] = {
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,            // P
    SEG_C | SEG_D | SEG_E | SEG_G,                    // o
    SEG_A | SEG_E | SEG_F | SEG_G,                    // f
    SEG_A | SEG_E | SEG_F | SEG_G                     // f
                                                      // Poff
};

const uint8_t S_o[] = {  
    SEG_C | SEG_D | SEG_E | SEG_G, 0x0,0x0, 0x0       // o                         
};

const uint8_t S_soff[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G,            // S
    SEG_C | SEG_D | SEG_E | SEG_G,                    // o
    SEG_A | SEG_E | SEG_F | SEG_G,                    // f
    SEG_A | SEG_E | SEG_F | SEG_G                     // f
                                                      // Soff
};

const uint8_t S_sin[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G,            // S
    SEG_E ,                                           // i
    SEG_C | SEG_E | SEG_G,                            // n
    SEG_D | SEG_E | SEG_F | SEG_G                     // t
};

const uint8_t S_out[] = {  //oth as in other
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G,            // S
    SEG_C | SEG_D | SEG_E | SEG_G,                    // o
    SEG_D | SEG_E | SEG_F | SEG_G,                    // t
    SEG_C | SEG_E | SEG_G | SEG_F                     // h 
};

const uint8_t S_stop[] = {
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G,            // S
    SEG_D | SEG_E | SEG_F | SEG_G,                    // t
    SEG_C | SEG_D | SEG_E | SEG_G,                    // o
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G             // p
};

const uint8_t S_play[] = {
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,            // P
    SEG_D | SEG_E | SEG_F,                            // L
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,    // A
    SEG_B | SEG_C | SEG_D | SEG_F | SEG_G             // y
};

const uint8_t S_hold[] = {
    SEG_C | SEG_E | SEG_G | SEG_F | SEG_B,            // H
    SEG_C | SEG_D | SEG_E | SEG_G,                    // o
    SEG_D | SEG_E | SEG_F,                            // L
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G             // d
};

const uint8_t S_dire[] = {
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,            // d
    SEG_E,                                            // i
    SEG_E | SEG_G,                                    // r
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G             // E
};

const uint8_t S_bpm[] = {
    SEG_C | SEG_D | SEG_E | SEG_F | SEG_G | SEG_A | SEG_B,    // b
    SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,                    // p
    SEG_C | SEG_E  | SEG_F | SEG_A | SEG_B,                   // n
    SEG_C |  SEG_A | SEG_B | SEG_F                            // m bit
};

const uint8_t S_ccon[] = {
    SEG_A | SEG_D| SEG_E | SEG_F,                     // C
    SEG_G | SEG_D | SEG_E,                            // c
    SEG_C | SEG_D | SEG_E | SEG_G,                    // o
    SEG_C | SEG_E | SEG_G                             // n
};

const uint8_t S_lbri[] = {
    SEG_D | SEG_E | SEG_F,                            // L
    SEG_C | SEG_D | SEG_E | SEG_F | SEG_G ,           // b
    SEG_E | SEG_G,                                    // r
    SEG_C                                             // i
};

const uint8_t S_L[] = {
    SEG_D | SEG_E | SEG_F,                            // L
    0x0,          
    0x0,                                   
    0x0                                            
};

const uint8_t S_offs[] = {
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,    // O
    SEG_A | SEG_E | SEG_F | SEG_G,                    // f
    SEG_A | SEG_E | SEG_F | SEG_G,                    // f
    SEG_A | SEG_D | SEG_C | SEG_F | SEG_G             // S
};

//assign digital pins
int EN = 8;         //enable pin
int S0 = 7;         //4 binary select pins
int S1 = 4;
int S2 = 5;
int S3 = 6;

int ledPin = 9;       // multiplexer LED connected to digital pin 9
int gatePin = 12;     // gate sync out connected to digital pin 12

void setup() {

  EEPROM_ADDRESS = 1;
  
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  
  pinMode(e_pin, INPUT_PULLUP);
  
  Serial.begin(31250);            //set for midi

  MIDI.begin(4);                      // Launch MIDI and listen to channel 4
  pinMode(ledPin, OUTPUT);

  midi_tap = 0;                                   //reset step
  step_cursor = 0;                                //reset step cursor
  p_state = 1;                                    //play state stop
  m_chan = 1;                                     //set midi channel to 1
  b_offset = 60;                                  //set bpm offset
  b_data = analogRead(A1)/256;                    //get analogue BPM to start

    if (b_data > 10) {                             //check if internal clock
      display.setSegments(S_b);                   //show b for BPM in 1st digit
      display.showNumberDec(bpm,false,3,1);       //show BPM value 
      
    } 
    else                                          //if internal show SYNC
    {display.setSegments(S_sync); bpm=0;}                   //show SYNC

  bpm = EEPROM.read(EEPROM_ADDRESS);
  c_data = EEPROM.read(2);
  s_data = EEPROM.read(3);
  o_data = EEPROM.read(4);
  brightness = EEPROM.read(5);

  
pinMode (EN, OUTPUT);           //encoder out (can be ground?)
pinMode (S0, OUTPUT);           //multiplexer pin select in binary
pinMode (S1, OUTPUT);
pinMode (S2, OUTPUT);
pinMode (S3, OUTPUT);
pinMode (gatePin, OUTPUT);      //set gate pin to output

digitalWrite(gatePin, HIGH);                  //first CV out pulse 1
//encoder setup
  pinMode(pinA, INPUT_PULLUP);    // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP);    // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

analogWrite(ledPin, (3+brightness*36));                              //multiplexer light on
display.setBrightness(brightness);                                   //set the diplay to brightness

  Timer1.initialize(intervalMicroSeconds);            //set interupt interval 
  Timer1.setPeriod(calculateIntervalMicroSecs(bpm));  
  Timer1.attachInterrupt(sendClockPulse);             //first pulse
  updateMode();                                       //starting mode
  stop_button();                                      //stop sequencer
}

void loop() {

if(oldEncPos != encoderPos) 
  {
  oldEncPos = encoderPos;
  }

unsigned long currentMillis = millis();
  
//pot_data = analogRead(A0);                //get analogue 1 data for parameter
//pot_data = map(pot_data, 0, 1023, 0, 127);
    
if (currentMillis - previousMillis >= 30) {
    // save the last time you checked
    previousMillis = currentMillis;
 
    //b_data = round(analogRead(A1)/170);                  //get analogue 2 data for select MODE (mapped for 7)
    b_data = map(analogRead(A1), 0, 1060,0, 7);
    //m_data = round(analogRead(A2)/64);                   //get analogue 3 data for MIDI channel (mapped for 16)
    m_data = map(analogRead(A7), 0, 1060,0, 11);
    
      if(digitalRead(e_pin) == HIGH) {  step_edit();}                                     //edit step mode
                                
      if(digitalRead(A3) == LOW) {play_button();}  else {p_butt=0;}                      //play button
      if(digitalRead(A4) == LOW) {stop_button();}  else {p_butt=0;}                      //stop and return to 1st step
      if(digitalRead(A5) == LOW) {hold_button();}  else {p_butt=0;}                      //hold button
      if(digitalRead(A2) == LOW) {select_button();}  else {p_butt=0;}                      //select button
      
    update_para();                                                      //show step parameter
}
  
  data = Serial.read();       //get midi data
  if(data == midi_clock) {    //if we get a click from midi in
    midi_tap ++;              //increment midi clock
    Serial.write(midi_clock); //send out midi click forward
  }

//encoder inputs
if(d_state == 1 and bpm !=  encoderPos) {             //update BPM
      updateBpm();
} 

if(d_state == 2 and ct_data !=  encoderPos) {          //update CC
      updateCC();
} 

if(d_state == 3 and s_data !=  encoderPos) {          //update SYNC
      updateSync();
}

if(d_state == 4 and shift_data !=  encoderPos) {      //update SHIFT
      updateShift();
}

if(d_state == 5 and o_data !=  encoderPos) {          //update Offset
      updateOffset();
}

if(d_state == 6 and brightness !=  encoderPos) {      //update Offset
      updateBright();
}

//potentiometer inputs 
  if(m_chan !=  m_data) {                             //update channel
      updateChan();
 } 

 if(mode !=  b_data) {                                //change encoder MODE
     updateMode();
 } 


}   //end of main loop


// SUBROUTINES


void selector(int cs)                                 //select multiplexor step
{

switch (cs){

  case 0:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, LOW);
    digitalWrite (S2, LOW);
    digitalWrite (S3, LOW);
    break;

  case 1:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, LOW);
    digitalWrite (S2, LOW);
    digitalWrite (S3, LOW);
    break;
    
  case 2:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, LOW);
    digitalWrite (S3, LOW);
    break;

  case 3:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, LOW);
    digitalWrite (S3, LOW);
    break;
    
   case 4:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, LOW);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, LOW);
    break;
    
   case 5:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, LOW);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, LOW);
    break;
    
   case 6:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, LOW);
    break;

  case 7:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, LOW);
    break;
    
  case 8:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, LOW);
    digitalWrite (S2, LOW);
    digitalWrite (S3, HIGH);
    break;
     
  case 9:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, LOW);
    digitalWrite (S2, LOW);
    digitalWrite (S3, HIGH);
    break;

  case 10:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, LOW);
    digitalWrite (S3, HIGH);
    break;

  case 11:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, LOW);
    digitalWrite (S3, HIGH);
    break;

  case 12:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, LOW);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, HIGH);
    break;
    
  case 13:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, LOW);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, HIGH);
    break;

  case 14:
    digitalWrite (EN, LOW);
    digitalWrite (S0, LOW);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, HIGH);
    break;
    
  case 15:
    digitalWrite (EN, LOW);
    digitalWrite (S0, HIGH);
    digitalWrite (S1, HIGH);
    digitalWrite (S2, HIGH);
    digitalWrite (S3, HIGH);
    break;
  }

}

void sendClockPulse() {
// Write the timing clock byte and increment sequencer 
Serial.write(midi_clock);     //send out midi click forward no matter what the play state is
//Serial.write(c_data);
if(p_state==0){                 //check play mode
  midi_tap ++;                  //increment midi clock
  pot_data = map(analogRead(A0), 0, 1020,0, 131);         //map to 128 TEST
  if(digitalRead(e_pin) == LOW) {msend = pot_data-4 ;} else {msend = step_v[step_tap]-4;}
  
  if(msend >= 0) {MIDI.sendControlChange( c_data, msend, m_chan);}   // send midi data

  if(midi_tap >= 6) {                                           //after 6 clicks move step forward

    if (digitalRead(gatePin) == LOW){ digitalWrite(gatePin,HIGH );} else {  digitalWrite(gatePin,LOW );}       // sends the cv out pulse
    
    midi_tap = 0;                                               //reset midi click
    if(digitalRead(e_pin) == LOW) {selector(step_tap);}         //only select channel if edit is off
    //if(msend >= 0) {MIDI.sendControlChange( c_data, msend, m_chan);}   // send midi data
    
    if(step_tap >= 15) {                                        //if at end of sequence (16) reset to start
        step_tap = 0;                                           //reset
        
         //if(msend >= 0) {MIDI.sendControlChange( 43, msend, m_chan);}  //send midi data out
         
        }        
    else  {step_tap++;  }
    //otherwise move step forward
  }
  
  }
}

long calculateIntervalMicroSecs(int bpm) {                      //calc interval for MIDI clock click
  // take care about overflows!
  return 60L * 1000 * 1000 / bpm / CLOCKS_PER_BEAT;
}

void updateBpm() {
   
    //selector(step_tap);                               //multiplexor select
    bpm = encoderPos;                                   //update BPM value

    if (bpm > 280) {bpm = 280; encoderPos = 280;}       //BPM ceiling
    if (bpm < 30) {bpm = 30; encoderPos = 30;}          //BPM floor
    
    if (bpm > 10) {                                     //check if internal clock
      display.setSegments(S_b);                         //show b for BPM in 1st digit
      display.showNumberDec(bpm,false,3,1);             //show BPM value 
      } 
    else                                                //if internal show SYNC
      {display.setSegments(S_sync); bpm=0;}             //show SYNC

   Timer1.setPeriod(calculateIntervalMicroSecs(bpm));   // Update the interval timer

  // Save the BPM
  EEPROM.write(EEPROM_ADDRESS, bpm);                    // Save BPM to eeprom
  bpmMillis = millis();                                 //last BPM change
 
}

void updateCC() {
   
    ct_data = encoderPos;                                     //update CC value

    if (ct_data > 128) {ct_data = 128; encoderPos = 128;}     //CC ceiling
    if (ct_data < 0) {ct_data = 0; encoderPos = 0;}           //CC floor

    if (c_data == ct_data){
      display.setSegments(S_c);                               //show C for cc in 1st digit
      display.showNumberDec(c_data,false,3,1);                //show CC value 
    }
    else
    {
      display.setSegments(S_c2);                              //show c for cc in 1st digit
      display.showNumberDec(ct_data,false,3,1);               //show CC selection value 
    }
    
  // Save the CC
  EEPROM.write(2, c_data);                                    // save CC to eeprom
  bpmMillis = millis();                                       // last timer change
}

void updateShift() {

    shift_data = encoderPos; 
    if (shift_data > 1) {shift_data = 1; encoderPos = 1; display.setSegments(S_plus); }             //sync ceiling
    if (shift_data < 0) {shift_data = 0; encoderPos = 0; display.setSegments(S_minus);}             //sync floor
 
  bpmMillis = millis();                                                                   // last timer change
}

void updateSync() {
   
    s_data = encoderPos;                                      //update sync value

    if (s_data > 2) {s_data = 2; encoderPos = 2;}             //sync ceiling
    if (s_data < 0) {s_data = 0; encoderPos = 0;}             //sync floor

    switch (s_data){
      case 0:                                                 //internal sync
        display.setSegments(S_sin);    
        break;
      case 1:                                                 //external sync
        display.setSegments(S_out);    
        break;
    }
      
  // Save the sync state
  EEPROM.write(3, s_data);                                    // save sync to eeprom
  bpmMillis = millis();                                       // last timer change
}

void updateOffset() {
   
    o_data = encoderPos;                                      //update offset value

    if (o_data > 64) {o_data = 64; encoderPos = 64;}          //offset ceiling
    if (o_data < -64) {o_data = -64; encoderPos = -64;}       //offset floor

    if (o_data >= 0){
      display.setSegments(S_o);                               //show o for offset in 1st digit
      display.showNumberDec(o_data,false,3,1);                //show offset value 
    }
    else
    {
      display.setSegments(S_omin);                            //show o for offset in 1st digit
      display.showNumberDec(o_data*-1,false,2,2);             //show offset value
    }
  // Save the offset
  EEPROM.write(4, o_data);                                    // save offset to eeprom
  bpmMillis = millis();                                       // last timer change
}

void updateBright() {
   
    brightness = encoderPos;                                      //update brightness value

    if (brightness > 7) {brightness = 7; encoderPos = 7;}         //brightness ceiling
    if (brightness < 0) {brightness = 0; encoderPos = 0;}         //brightness floor

    analogWrite(ledPin, (3+brightness*36));                       //multiplexer light on
    display.setBrightness(brightness);                            //set the diplay to maximum brightness

      display.setSegments(S_L);                                   //show o for brightness in 1st digit
      display.showNumberDec(brightness,false,3,1);                //show brightness value 
  
  // Save the brightness
  EEPROM.write(5, brightness);                                    // save brightness to eeprom
  
  bpmMillis = millis();                                           // last timer change
}

void updateMode() {

    mode = b_data;                                       //update mode to analogue input

    switch (mode){
    case 0:
      display.setSegments(S_bpm);
      encoderPos = bpm;                                 //set encoder to BPM
      d_state = 1;
      break;
    case 1:
      display.setSegments(S_ccon);
      encoderPos = c_data;                              //set encoder to CC
      ct_data = c_data;                                  //set select cc to same
      d_state = 2;
      break;
    case 2:
      display.setSegments(S_sync);                      //set sync
      encoderPos = s_data;
      d_state = 3; 
      break;
    case 3:
      display.setSegments(S_shft);                      //shift pulses
      encoderPos = shift_data;
      d_state = 4;
      break;
    case 4:
      display.setSegments(S_offs);                      //offset parameter display
      encoderPos = o_data;
      d_state = 5;
      break;
    case 5:
      display.setSegments(S_lbri);                      //set display brightness
      encoderPos = brightness;
      d_state = 6;
      break;
    case 6:
      display.setSegments(S_dire);                      //set display brightness
      encoderPos = brightness;
      d_state = 7;
      break;
    }

  bpmMillis = millis();                                     //last BPM change
  
}

void updateChan() {

    m_chan = m_data;                                        //update midi channel value
      display.setSegments(S_ch);                            //show ch
      if (m_chan != 0){
        display.showNumberDec(m_chan,false,2,2);              //show channel value 
      }
      else
      {
        display.setSegments(S_no);                         //show no channel
      }
  bpmMillis = millis();                                     //last timer change
}


//rotary encoder stuff


void PinA(){
  cli();                                  //stop interrupts happening before we read pin values
  reading = PIND & 0xC;                   // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) {     //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    
    if(digitalRead(e_pin) == HIGH) {encoderStep --;} else {encoderPos --;}                        //decrement the encoder's position count
    bFlag = 0;                            //reset flags for the next turn
    aFlag = 0;                            //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei();                                    //restart interrupts
}

void PinB(){
  cli();                                  //stop interrupts happening before we read pin values
  reading = PIND & 0xC;                   //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) {    //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    
    if(digitalRead(e_pin) == HIGH) {encoderStep ++;} else {encoderPos ++;}                        //increment the encoder's position count
    bFlag = 0;                            //reset flags for the next turn
    aFlag = 0;                            //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei();                                    //restart interrupts
}

void update_para(){
    if(digitalRead(e_pin) == LOW) {selector(step_tap);        // multiplexor select
         if(millis() - bpmMillis  > 700){                     //check if something else is hogging screen
            
            pot_data = analogRead(A0);                        //get analogue 1 data for parameter
            pot_data = map(pot_data, 0, 1020,0, 131);         //map to 128
            if(pot_data>3)               
              {

                if ((pot_data-3-o_data) >= 0){
                  if(digitalRead(e_pin) == LOW){ display.setSegments(S_p);}     //show P for parameter in 1st digit
                  display.showNumberDec(pot_data-3-o_data,false,3,1);                  //show parameter value 
                }
                else
                {
                  if(digitalRead(e_pin) == LOW){ display.setSegments(S_pmin);}     //show P for parameter in 1st digit with minus
                  display.showNumberDec((pot_data-3-o_data)*-1,false,2,2);                  //show parameter value 
                }
              
              }
            else
              if(digitalRead(e_pin) == LOW){ {display.setSegments(S_off);}
              }                                                               //show Poff for parameter
         } 
    }
}

void play_button(){
  if (p_state !=0 ){                                               //check button state

//  if(p_state==0){
//    display.setSegments(S_hold);bpmMillis = millis();         //pause sequence (hold)
//    p_state=1;
//    }
//  else
  //  {
    display.setSegments(S_play);bpmMillis = millis();           //play sequence
    p_state=0;
    Serial.write(midi_start);                                   //send out midi start signal
 //   }
 p_butt=1;                                                      //set play button pressed (off to reset)
 }
}

void stop_button(){
  if (p_state !=2 ){                                            //check button state
    display.setSegments(S_stop);bpmMillis = millis();           //stop sequence
    p_state=2;
    Serial.write(midi_stop);                                    //send out midi stop signal
midi_tap = 0;                                                   //set seqencer steps to zero
step_tap =0;
p_butt=1;                                                       //set play button pressed (off to reset)
  }
}

void hold_button(){
  if (p_butt==0){                                               //check button state

    display.setSegments(S_hold);bpmMillis = millis();           //hold sequence
    p_state=1;
p_butt=1;                                                       //set play button pressed (off to reset)
  }
}

void select_button(){

if (d_state == 1){                                              //check if in BPM mode
  updateBpm();                                                  //update BPM to show current value
}

if (d_state == 2){                                              //check if in CC mode
  c_data = ct_data;                                             //update CC to selection
  updateCC();                                                   //update CC
}

if (d_state == 3){                                              //check if in sync mode
    switch (s_data){
      case 0:                                                   //internal sync
        display.setSegments(S_sin);    
        break;
      case 1:                                                   //external sync
        display.setSegments(S_out);    
        break;
    }
    bpmMillis = millis();  
}

if (d_state == 4){                                                        //check if in shift mode
  if (shift_data == 0) {step_tap--; display.setSegments(S_minus);}        //shift back
  if (shift_data == 1) {midi_tap++; display.setSegments(S_plus);}         //shift forward 
  if(midi_tap >= 6) {midi_tap = 0; step_tap ++;}
  bpmMillis = millis();         
}

if (d_state == 5){                                                        //check if in offset mode
  updateOffset();                                                         //update offset to show current value  
}

if (d_state == 6){                                                        //check if in Brightness mode
  updateBright();                                                         //update Brightness to show current value  
}

}

void step_edit(){
//show edit cursor
//bpmMillis = millis();
step_cursor = encoderStep;
    if (step_cursor > 16) {step_cursor = 1; encoderStep = 1;}           //step ceiling
    if (step_cursor < 1) {step_cursor = 16; encoderStep = 16;}          //step floor
selector(step_cursor-1);                                                //select multiplexor for step cursor
pot_data = analogRead(A0);                                              //get analogue 1 data for parameter

        pot_data = map(pot_data, 0, 1020,0, 131);             //map to 128
            if(pot_data > 3)              
              {
                //display.setSegments(S_S);                     //show P for parameter in 1st digit
                //display.showNumberDec(pot_data-3,false,3,1);  //show parameter value for cursor
                
                if ((pot_data-3-o_data) >= 0){
                  display.setSegments(S_S);                                         //show S for parameter in 1st digit
                  display.showNumberDec(pot_data-3-o_data,false,3,1);               //show parameter value for cursor
                }
                else
                {
                  display.setSegments(S_Smin);                                      //show S for parameter in 1st digit with minus
                  display.showNumberDec((pot_data-3-o_data)*-1,false,2,2);          //show parameter value for cursor
                }
              }
            else
              {display.setSegments(S_soff);}                  //show Soff for cursor
              step_v[step_tap] = pot_data;                    //step value to array
if(digitalRead(e_pin) == LOW) {updateMode();}                 //show mode on release
}


 
