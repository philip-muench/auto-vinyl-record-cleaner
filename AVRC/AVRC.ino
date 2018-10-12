#include <Wire.h>
#include <Servo.h>
#include <math.h>

// Phases
#define WAITING 0
#define PREPARE 1
#define SWING_IN 2
#define CLEAN 3
#define GIVE_YARN_GRAB 4
#define GIVE_YARN_UNCOIL 5
#define GIVE_YARN_RETURN 6
#define GIVE_YARN_TAUTEN 7
#define SWING_OUT 8

// Phase durations / timing 

// Alle Zeiten sind in Millisekunden angegeben.
// Alle Zeiten bis auf die letzte geben die Zeitdauer der jeweiligen Phase an.
// Die letzte Zeit gibt an, wie lange der Servo benoetigen soll, um sich ein Grad weiterzubewegen.
#define PREPARE_TIME 1000
#define CLEAN_TIME 120000
#define GIVE_YARN_GRAB_TIME 500
#define GIVE_YARN_UNCOIL_TIME 500
#define GIVE_YARN_RETURN_TIME 500
#define GIVE_YARN_TAUTEN_TIME 500

//Displaytexte fuer die Phasen
const char prepare_text[] =            "Automatik ueber-nimmt Steuerung";
const char swing_in_text[] =           "AUTO: Arm in    Startposition";
const char clean_text[] =              "AUTO: Pumpe ein,Ruecklauf Arm";
const char swing_out_text[] =          "Arm in Ruheposi-tion bringen";
const char give_yarn_grab_text[] =     "Fadenvorschub";
const char give_yarn_uncoil_text[] =   "Fadenvorschub";
const char give_yarn_return_text[] =   "Fadenvorschub";
const char give_yarn_tauten_text[] =   "Faden spannen";
const char waiting_text[] =            "manueller Modus:vor/rueck-U/min.";

const char* texts[9];


// IO Pins
#define PUMP 2
#define CONTROL 3
#define STEPPER_P1 4
#define STEPPER_P2 5
#define STEPPER_P3 6
#define STEPPER_P4 7
#define YARN_MAG 8
#define YARN_SERVO 9
#define MOTOR 10

// Pump arm config
#define MIN_ANGLE 18
#define MAX_ANGLE 49
#define DEG_DIV 10
#define TRANSMISSION 5
#define RATIO 1.5

//define output bitfield
#define PUMP_BIT 1
#define YM_BIT 2
#define DRV_BIT 4

//Variables
Servo srv;
unsigned int phase = WAITING;
unsigned long time = 0;
unsigned long timing;
int _stepouts[] = {1, 3, 2, 6, 4, 12, 8, 9};
int _step = 0;
float mysperstep;
float clean_step;
float delta;
float timeToNextPause;
float stepsperstep;
long next_step;
volatile bool buttonPressed;
int outputConfig[9];

long steps_taken = 0;
long start_steps = MAX_ANGLE * TRANSMISSION * (4096.0/360.0);
long stop_steps = MIN_ANGLE * TRANSMISSION * (4096.0/360.0);

void setupOutputConfig() {
  outputConfig[WAITING]          = 0;
  outputConfig[PREPARE]          = DRV_BIT;
  outputConfig[SWING_IN]         = DRV_BIT;
  outputConfig[CLEAN]            = DRV_BIT | PUMP_BIT;
  outputConfig[SWING_OUT]        = DRV_BIT;
  outputConfig[GIVE_YARN_GRAB]   = YM_BIT;
  outputConfig[GIVE_YARN_UNCOIL] = YM_BIT;
  outputConfig[GIVE_YARN_RETURN] = YM_BIT;
  outputConfig[GIVE_YARN_TAUTEN] = PUMP_BIT;
}

void setupTexts()
{
  texts[WAITING] = waiting_text;
  texts[PREPARE] = prepare_text;
  texts[SWING_IN] = swing_in_text;
  texts[CLEAN] = clean_text;
  texts[SWING_OUT] = swing_out_text;
  texts[GIVE_YARN_GRAB] = give_yarn_grab_text;
  texts[GIVE_YARN_UNCOIL] = give_yarn_uncoil_text;
  texts[GIVE_YARN_RETURN] = give_yarn_return_text;
  texts[GIVE_YARN_TAUTEN] = give_yarn_tauten_text;
}

void setupTiming() 
{
  long steps = (MAX_ANGLE - MIN_ANGLE) * DEG_DIV * TRANSMISSION;
  float summ = 0.0;
  float summand = 1.0;
  delta = (RATIO - 1.0) / (float)steps;
  for (int i = 0; i < steps; i++)
  {
    summ += summand;
    summand += delta;
  }
  mysperstep = ((float)(CLEAN_TIME * 1000.0)) / summ;
  Serial.println(summ);
  Serial.println(delta * 1000.0);
  Serial.println(mysperstep);
  Serial.println(steps);
  clean_step = 1.0;
  
  stepsperstep = ((float)(start_steps - stop_steps))/((float)steps);
  Serial.println(stepsperstep);
  Serial.println(stepsperstep * steps);
  Serial.println(start_steps - stop_steps);
}

float timeToNextMove;


void init_clean() 
{
  timeToNextPause = mysperstep/1000.0;
  clean_step = 1.0;
  timeToNextMove = timeToNextPause - stepsperstep; 
}  

void advance_clean() 
{ 
  clean_step += delta;
  float temp = (mysperstep * clean_step);
  timeToNextPause = temp/1000.0;
  timeToNextMove = timeToNextPause - stepsperstep;
}  


void setup() {
  
  pinMode(PUMP, OUTPUT);
  
  pinMode(CONTROL, INPUT_PULLUP);
  attachInterrupt(1, buttonInterrupt, RISING);
  
  pinMode(STEPPER_P1, OUTPUT);
  pinMode(STEPPER_P2, OUTPUT);
  pinMode(STEPPER_P3, OUTPUT);
  pinMode(STEPPER_P4, OUTPUT);
  stop();
  
  pinMode(YARN_MAG, OUTPUT);
 
  pinMode(MOTOR, OUTPUT);
 
  srv.attach(YARN_SERVO);  
  srv.write(90);
  
  Serial.begin(9600);
  
  setupDisp();
  writeWord("Bitte warten");
  
  setupTiming();
  setupOutputConfig();
  setupTexts();
   
  delay(5000);
  resetInterrupt();
  setPhase(WAITING);
  timing = micros();
}

void buttonInterrupt() {
  buttonPressed = true;
}
void resetInterrupt() {
  buttonPressed = false;
}



void setPhase(int newphase) {
  phase = newphase;
  int opconf = outputConfig[newphase];
  Serial.print("PUMP BIT: ");
  Serial.println(opconf & PUMP_BIT ? "true" : "false");
  Serial.print("MOTOR BIT: ");
  Serial.println(opconf & DRV_BIT ? "true" : "false");
  Serial.print("YARN MAGNET BIT: ");
  Serial.println(opconf & YM_BIT ? "true" : "false");
  digitalWrite(PUMP, opconf & PUMP_BIT ? HIGH : LOW);
  digitalWrite(MOTOR, opconf & DRV_BIT ? HIGH : LOW);
  digitalWrite(YARN_MAG, opconf & YM_BIT ? HIGH : LOW);
  Serial.println(texts[newphase]);
  writeWord(texts[newphase]);
  resetTime();
}


bool checkCancel() {
  if(buttonPressed){
    setPhase(SWING_OUT);
    Serial.println("Button pressed to cancel. Next phase SWING_OUT.");
    return true;
  }
  return false;
}

void incrTime() {
  time += 1;
  delay(1);
}

void resetTime() {
  time = 0;
}

void setupDisp() {
  delay(50);
  Wire.begin();
  for (int i = 0; i < 2; i++) {
    writeByte(48);
    delay(5);
    writeByte(48);
    delay(5);
    writeCommand(34);
    writeCommand(40);
    writeCommand(2);
    delay(5);
    writeCommand(1);
    writeCommand(12);
  } 
}

void writeByte(int b){
  b = b | 8;
  Wire.beginTransmission(0x27);
  Wire.write(b);
  delayMicroseconds(1);
  Wire.write(b|4);
  delayMicroseconds(1);
  Wire.write(b);
  delayMicroseconds(100);
  Wire.endTransmission();
}

void writeCommand(int c){
  int low = (c & 0x0F)<<4;
  int high = (c & 0xF0);
  writeByte(high);
  writeByte(low);
  delayMicroseconds(50);
}

void writeWord(const char* s) { 
  writeCommand(8);
  writeCommand(1); 
  writeCommand(2);
  delay(5);
  char c;
  int pos = 0;
  while((c=s[pos++])!= 0){
    writeData((int) c);
    if(pos == 16) writeCommand(192);
  }
  writeCommand(12);
}

void writeData(int c){
  int low = (c & 0x0F)<<4;
  int high = (c & 0xF0);
  writeByte(high | 1);
  writeByte(low | 1);
  delayMicroseconds(50);
}

void step_write(int b)
{
  for(int i = 0; i < 4; i++) 
  {
    digitalWrite(i+4, b & (1 << i) ? HIGH : LOW);
  }  
}

void stepForward()
{
  steps_taken++;
  _step = (_step + 1) % 8;
  step_write(_stepouts[_step]);
}

void stepBackward()
{
  steps_taken--;
  _step = (_step + 7) % 8;
  step_write(_stepouts[_step]);
}

void stop() 
{
  step_write(0);
}


long startTime;
void loop() {
  // put your main code here, to run repeatedly:
  switch(phase){

    case WAITING:
      if(buttonPressed){
        setPhase(PREPARE);
      }
      break;
      
    case PREPARE:
      if(time > PREPARE_TIME){
        setPhase(SWING_IN);
      }
      break;
      
    case SWING_IN:
      if(checkCancel()) break;
      if(steps_taken >= start_steps)
      {
        setPhase(CLEAN);
        init_clean();
        stop();
        startTime = millis();
        break;
      }
      stepForward();
      break;
      
    case CLEAN:
      if(checkCancel()) break;
      if(steps_taken <= stop_steps)
      {
        Serial.println(millis() - startTime);
        setPhase(SWING_OUT);
        break;
      }
      if(time >= (int)timeToNextPause)
      {
        //stop();
        advance_clean();
        resetTime();
        break;
      }
      if(time >= (int)timeToNextMove)
      {
        stepBackward();
      }
      break;
      
    case SWING_OUT:
      if(steps_taken <= 0)
      {
        setPhase(GIVE_YARN_GRAB);
        //stop();
        break;
      }
      stepBackward();
      break;
      
    case GIVE_YARN_GRAB:
      if(time > GIVE_YARN_GRAB_TIME){
        setPhase(GIVE_YARN_UNCOIL);
      }
      break;
      
    case GIVE_YARN_UNCOIL:
      srv.write(45);
      if(time > GIVE_YARN_UNCOIL_TIME){
        setPhase(GIVE_YARN_RETURN);
      }
      break;
      
    case GIVE_YARN_RETURN: 
      srv.write(90);
      if(time > GIVE_YARN_RETURN_TIME){
        setPhase(GIVE_YARN_TAUTEN);
      }
      break;
      
    case GIVE_YARN_TAUTEN:
      if(time > GIVE_YARN_TAUTEN_TIME){
        setPhase(WAITING);
      }
      break;
      
    default:
      break;
  }
  resetInterrupt();
  incrTime();
}


