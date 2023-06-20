/* 
LineFollowerQTR8A
Adrie Huesman
24 September 2021

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
QTR-8A Reflectance Sensor Array Pololu 960 and SparkFun Multiplexer Breakout - 8 Channel (74HC4051)
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
D7 --
D8 --
D9 Servo
D10 Neopixel
D11 74HC4051 S0 
D12 74HC4051 S1
D13 74HC4051 S2
A0 Right IR sensor
A1 Left IR sensor
A2 QTR-8A via multiplexer
A3 --
A4 --
A5 --

Other
One wheel turn is 120*3 = 360 steps or pi*60 = 188.5 mm. So 0.5236 mm/step.
A 90 degree turn is pi*167/4 = 131.2 mm. So 250.6 steps.

SensorValues when all black 958 938 932 934 935 945 948 962
SensorValues when all white 318 130 82  112 90  178 184 306 
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_NeoPixel.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4
Adafruit_DCMotor *RMotor = AFMS.getMotor(3);
Adafruit_DCMotor *LMotor = AFMS.getMotor(4);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, 10, NEO_GRB + NEO_KHZ800);

// Declare variables
float LinePosition;
float Error;   
float Kc = 30.0;   // Controller gain (20.0)
float Paction;
int mspeed = 100;   // Average speed (75)
int Rspeed;
int Lspeed;
uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t yellow = strip.Color(128, 128, 0);
uint32_t lightblue = strip.Color(0, 128, 128);
uint32_t magenta = strip.Color(128, 0, 128);
uint32_t white = strip.Color(85, 85, 85);

// Function to determine line position
float Position() 
{
  int MinValues[] = {318, 130, 82,  112, 90,  178, 184, 306}; // All sensors facing white surface, basically calibration
  int RawValues[8];  // Direct measured values
  int ShiftValues[8];  // Values minus MinValues but >= 0
  int SumValue;  // Sum of all ShiftValues
  int WSValue;  // Sum of all position weighted ShiftValues
  float Pos;  // Line position
  // Get raw values first
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  RawValues[0] = analogRead(A2);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  RawValues[1] = analogRead(A2);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
  RawValues[2] = analogRead(A2);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
  RawValues[3] = analogRead(A2);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  RawValues[4] = analogRead(A2);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(13, HIGH);
  RawValues[5] = analogRead(A2);
  digitalWrite(11, LOW);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  RawValues[6] = analogRead(A2);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  RawValues[7] = analogRead(A2);
  // Now calculate line position
  SumValue = 0;
  WSValue = 0; 
  for (int i = 0; i <= 7; i++)
  {
    ShiftValues[i] = max(0, (RawValues[i] - MinValues[i])); 
    SumValue = SumValue + ShiftValues[i];
    WSValue = WSValue + (i + 1)*ShiftValues[i]; 
  }
  if (SumValue <= 80)
  {
    Pos = 0; // Signal too low, basically no line detected
  }
  if (SumValue > 80)
  {
    Pos = float(WSValue)/float(SumValue);
    Pos = 9 - Pos; // Converting left to 1 and right to 8
  }
  return Pos;
}    

// Function to make R2D2 sound
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
  Serial.begin(9600);  // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  strip.begin();
  strip.setBrightness(150);
  strip.show(); // Initialize all pixels to 'off'

  randomSeed(analogRead(A3));

  // Some light and sound effects
  strip.fill(red, 0, 12);
  strip.show();
  delay(1000);
  strip.fill(blue, 0, 12);
  strip.show();
  delay(1000);
  strip.fill(green, 0, 12);
  strip.show();
  delay(1000);
  strip.clear();
  strip.show();
  R2D2();
}

void loop()
{
  LinePosition = Position(); 
  // Serial.println(LinePosition);
  if (1.0 <= LinePosition && LinePosition <= 8.0)
  {
    Error = 4.50 - LinePosition;
    Paction = int(Kc*Error);
    Rspeed = mspeed + Paction;
    Lspeed = mspeed - Paction;
    RMotor->run(FORWARD);
    LMotor->run(FORWARD);
    RMotor->setSpeed(Rspeed);
    LMotor->setSpeed(Lspeed);  
  }
  if (LinePosition <= 0.5)
  {
    RMotor->run(FORWARD);
    LMotor->run(FORWARD);
    RMotor->setSpeed(0);
    LMotor->setSpeed(0);  
  }
}  
