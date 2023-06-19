/* 
RoverB
Adrie Huesman
1 September 2021

Hardware
Arduino Uno
Adafruit motor shield V2
Rechargeable NiMH Battery Pack: 6.0V, 2200mAh, 5x1AA Cells, JR Connector Pololu 2223
120:1 Mini Plastic Gearmotor HP, Offset 3mm D-Shaft Output, Extended Motor Shaft Pololu 1520
Magnetic Encoder Pair Kit for Mini Plastic Gearmotors, 12 CPR, 2.7-18V Pololu 1523
Wheel 60x8mm Pair Pololu 1420/1421/1424
Powerboard, own design based on 5V Step-Up/Step-Down Voltage Regulator S7V7F5 Pololu 2119g
Servo SG-5010 (modified with 2K resitors) and ultrasonic distance sensor HC-SR04
Two IR Sharp distance sensors GP2Y0A41SK0F
NeoPixel Stick 8 x WS2812 5050 RGB LED Adafruit 1426
Passive buzzer HW-508
Chassis, own design 4mm aviation plywood 17cm by 12 cm (two levels)

Software
Make use of R2D2 sound by guiaarduino, see // https://ar2d2uino.wordpress.com/2016/02/21/construa-seu-proprio-r2d2-com-arduino/
Inspired by https://dronebotworkshop.com/robot-car-with-speed-sensors/
Based on own Raspberry Pi robots, also see https://www.norwegiancreations.com/2017/03/state-machines-and-arduino-implementation/

I/O
Adfruit motor shield works via I2C
Right motor connected to M3
Left motor connected to M4
D0 --
D1 --
D2 Right encoder
D3 Left encoder
D4 Ultrasonic sensor trigger
D5 Ultrasonic sensor echo
D6 Passive buzzer
D7 Neopixel
D8 --
D9 Servo
D10 --
D11 --
D12 --
D13 --
A0 Right IR sensor
A1 Left IR sensor
A2 --
A3 --
A4 --
A5 --

Other
One wheel turn is 120*3 = 360 steps or pi*60 = 188.5 mm. So 0.5236 mm/step.
A 90 degree turn is pi*167/4 = 131.2 mm. So 250.6 steps. 
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h> 
#include <HCSR04.h>
#include <Adafruit_NeoPixel.h>

Servo myservo;
int right = 940;    // 940
int center = 1450;  // 1450
int left = 1950;    // 1950

// Initialize sensor that uses digital pins 13 and 12.
const byte triggerPin = 4;
const byte echoPin = 5;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4
Adafruit_DCMotor *RMotor = AFMS.getMotor(3);
Adafruit_DCMotor *LMotor = AFMS.getMotor(4);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, 10, NEO_GRB + NEO_KHZ800);

// Declare variables
int aspeed = 60; // average speed when driving forward
int Error;
float Kc = 1.0;
float Paction;
int Rspeed;
int Lspeed;
bool Rrun;
bool Lrun;
enum state
{
  INI,
  SAM,
  MFO,
  T90,
  T15,
  ESC
};
state Robotstate = INI;
int sensorPinR = A0;    // select the input pin for sensor
int Rsignal;            // variable to store the value coming from right sensor
int sensorPinL = A1;    // select the input pin for sensor
int Lsignal;          // variable to store the value coming from left sensor
float Cdistance;
int Llimit = 165;       // 0.8 volt around 16 cm
int Rlimit = 165;
float Climit = 22.0;    //  22.0 cm to avoid T15 after T90
byte Lcontr;
byte Rcontr;
byte Ccontr;
byte LRC;
float Ldistance;
float Rdistance;
uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t yellow = strip.Color(128, 128, 0);
uint32_t lightblue = strip.Color(0, 128, 128);
uint32_t magenta = strip.Color(128, 0, 128);
uint32_t white = strip.Color(85, 85, 85);

// Constants for interrupt pins for Arduino Uno
const byte RMotorpin = 2;  // Right Motor Interrupt Pin - INT 0
const byte LMotorpin = 3;  // Left Motor Interrupt Pin - INT 1
 
// Integers for pulse counters
volatile int Rcounter = 0;
volatile int Lcounter = 0;
 
// Interrupt Service Routines
 
// Right Motor pulse count ISR
void ISR_countR()  
{
  Rcounter++;  // increment Right Motor counter value
} 
 
// Left Motor pulse count ISR
void ISR_countL()  
{
  Lcounter++;  // increment Left Motor counter value
} 

// Function to Move
void Move(char dir, int steps, int mspeed) 
{
  // Reset counters
  Rcounter = 0;
  Lcounter = 0;
  Rspeed = mspeed;
  Lspeed = mspeed;
  Rrun = true;
  Lrun = true;
  // Set direction motors
  if (dir == 'F')
  {
    RMotor->run(FORWARD);
    LMotor->run(FORWARD);
  }
  if (dir == 'B')
  {
    RMotor->run(BACKWARD);
    LMotor->run(BACKWARD);
  }
  if (dir == 'R')
  {
    RMotor->run(BACKWARD);
    LMotor->run(FORWARD);
  }
  if (dir == 'L')
  {
    RMotor->run(FORWARD);
    LMotor->run(BACKWARD);
  }
  RMotor->setSpeed(Rspeed);
  LMotor->setSpeed(Lspeed);  
  // Run motors under P control until step values are reached
  do
  {
    Error = Rcounter - Lcounter;
    Paction = Kc*Error;
    Rspeed = mspeed - int(Paction);
    Lspeed = mspeed + int(Paction);  
    RMotor->setSpeed(Rspeed);
    LMotor->setSpeed(Lspeed);  
    if (Rcounter >= steps)
    {
      RMotor->setSpeed(0);
      Rrun = false;
    }
    if (Lcounter >= steps)
    {
      LMotor->setSpeed(0); 
      Lrun = false;      
    }
  } while (Rrun || Lrun);
    // Serial.println(Rcounter);
    // Serial.println(Lcounter);
}    

void R2D2()
{
  for (int i = 1; i <= 6; i++)
  {
    int noteRandom = random(50, 4250);
    Serial.println(noteRandom);
    tone(6, noteRandom, 200);
    delay(100);
    noTone(6);
  }
}  

void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  // Serial.println("Setup");

  myservo.attach(9);

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  pinMode(RMotorpin, INPUT_PULLUP);  
  pinMode(LMotorpin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt (RMotorpin), ISR_countR, FALLING);  // Increase right counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (LMotorpin), ISR_countL, FALLING);  // Increase left counter when speed sensor pin goes High

  strip.begin();
  strip.setBrightness(150);
  strip.show(); // Initialize all pixels to 'off'

  randomSeed(analogRead(2));
}

void loop()
{
  // Serial.println("Loop");
  switch(Robotstate)
  {
    case INI:
    strip.fill(white, 0, 12);
    strip.show();
    R2D2();
    delay(1000);
    RMotor->run(FORWARD);
    LMotor->run(FORWARD);
    RMotor->setSpeed(0);
    LMotor->setSpeed(0);  
    myservo.writeMicroseconds(center);  // set servo to center position
    delay(2000);
    Robotstate = SAM;
    strip.clear();
    strip.show();
    break;
       
    case SAM:
    strip.fill(blue, 0, 12);
    strip.show();
    RMotor->run(FORWARD);
    LMotor->run(FORWARD);
    RMotor->setSpeed(0);
    LMotor->setSpeed(0);  
    myservo.writeMicroseconds(center);  // set servo to center position
    delay(1000);
    Lsignal = analogRead(sensorPinL);
    Rsignal = analogRead(sensorPinR);
    Cdistance = distanceSensor.measureDistanceCm();
    if (Lsignal > Llimit)
    {
      Lcontr = 4;
    }
    else
    {
      Lcontr = 0;
    }
    if (Rsignal > Rlimit)
    {
      Rcontr = 2;
    }
    else
    {
      Rcontr = 0;
    }
    if (Cdistance < Climit)
    {
      Ccontr = 1;
    }
    else
    {
      Ccontr = 0;
    }
    LRC = Lcontr + Rcontr + Ccontr;
    // Serial.println(LRC);
    if (LRC == 0)
    {
      Robotstate = MFO;
    }
    if (LRC == 1)
    {
      Robotstate = T90;
    }
    if (LRC >= 2 && LRC <= 5)
    {
      Robotstate = T15;
    }
    if (LRC == 6 || LRC == 7)
    {
      Robotstate = ESC;
    }
    strip.clear();
    strip.show();
    break;

    case MFO:
    strip.fill(green, 0, 12);
    strip.show();
    RMotor->run(FORWARD);
    LMotor->run(FORWARD);
    Error = Rcounter - Lcounter;
    Paction = Kc*Error;
    Rspeed = aspeed - int(Paction);
    Lspeed = aspeed + int(Paction);  
    RMotor->setSpeed(Rspeed);
    LMotor->setSpeed(Lspeed);
    delay(100);
    Lsignal = analogRead(sensorPinL);
    Rsignal = analogRead(sensorPinR);
    Cdistance = distanceSensor.measureDistanceCm();
    if (Lsignal > Llimit)
    {
      Lcontr = 4;
    }
    else
    {
      Lcontr = 0;
    }
    if (Rsignal > Rlimit)
    {
      Rcontr = 2;
    }
    else
    {
      Rcontr = 0;
    }
    if (Cdistance < Climit)
    {
      Ccontr = 1;
    }
    else
    {
      Ccontr = 0;
    }
    LRC = Lcontr + Rcontr + Ccontr;
    if (LRC == 0)
    {
      Robotstate = MFO;
    }
    if (LRC != 0)
    {
      Robotstate = SAM;
    }
    if (Cdistance >= 100.0)
    {
      aspeed = aspeed + 2;
      aspeed = min(aspeed, 120);
    }
    if (Cdistance <= 50.0)
    {
      aspeed = aspeed - 4;
      aspeed = max(aspeed, 60);
    }
    strip.clear();
    strip.show();
    break;

    case T90:
    strip.fill(magenta, 0, 12);
    strip.show();
    R2D2();
    myservo.writeMicroseconds(left);
    delay(1000);
    Ldistance = distanceSensor.measureDistanceCm();
    myservo.writeMicroseconds(right);
    delay(2000);
    Rdistance = distanceSensor.measureDistanceCm();
    myservo.writeMicroseconds(center);
    delay(1000);
    if (Ldistance >= Rdistance)
    {
      Move('L', 251, 50);
    }
    if (Rdistance > Ldistance)
    {
      Move('R', 251, 50);
    }
    Robotstate = SAM;
    strip.clear();
    strip.show();
    break;

    case T15:
    strip.fill(yellow, 0, 12);
    strip.show();
    if (LRC == 2 || LRC == 3)
    {
      Move('L', 42, 50);;
    }
    if (LRC == 4 || LRC == 5)
    {
      Move('R', 42, 50);
    }
    Robotstate = SAM;
    strip.clear();
    strip.show();
    break;

    case ESC:
    strip.fill(red, 0, 12);
    strip.show();
    Move('B', 575, 60);
    Move('R', 502, 50);
    Robotstate = SAM;
    strip.clear();
    strip.show();
    break;
  }
  
}  
