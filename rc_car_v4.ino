/*
 * Project: Arduino RC Car with PPM Receiver with iBUS battery tension feedback
 * Autor: Pedro Igo Sousa Lucas
 */

#include <Servo.h>
#include <AFMotor.h>
#include <PPMReader.h>  // By Nikkilae (Access Mar 10th 2020 https://github.com/Nikkilae/PPM-reader)
#include <InterruptHandler.h>  // By zeitgeist87 (Access Mar 10th 2020 https://github.com/zeitgeist87/InterruptHandler)
#include <iBUStelemetry.h>  // By Hrastovc (Access Mar 10th 2020 https://github.com/Hrastovc/iBUStelemetry)

#define channelAmount 5 // Number of channels

// Motors declaration. AFMotor library
AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

double inputValue [channelAmount];

double vl, vr; // Motor velocities
// Motor Factors. Adjust to compensate for weaker motors. Max = 200, Min = 0
// setSpeed can receive 0 - 255, but i limited to 200 to avoid motor strain
// OBS: Only necessary if your controller doesn't have trim
double vlf = 250;
double vrf = 250;

int ledPin = 14; // Eye led's pin
int buzzer = 18;
int tension = 19;
int servoPin = 9; 

Servo head;

//Receiver pin (must be an interrupt capable pin)
int interruptPin = 2;
PPMReader ppm(interruptPin, channelAmount);

//Channels
double lr; // Left / Right
double fb; // Forward / Back
double bz; //buzzer
double sv; // Servo
double ts; // Turn style

//Receiver input assignment
void setup() 
{
  pinMode (tension, INPUT);
  pinMode(ledPin, OUTPUT); pinMode(buzzer, OUTPUT);
  
  head.attach(servoPin);
  
  Serial.begin(9600); 
  
  telemetry.begin(115200);
  telemetry.addSensor(0);
}

void loop() 
{
  // input receiving (PPM reading)
  // OBS: Pin A0 / 14 has weak solder and keeps losing contact
  for (int channel = 1; channel <= channelAmount; ++channel)
  {
    inputValue[channel - 1] = ppm.latestValidChannelValue (channel, 0);
  }
  lr = inputValue [0];
  fb = inputValue [1];
  bz = inputValue [2];
  sv = inputValue [3];
  ts = inputValue [4];

  voice();
  if (lr > 1)
  {
    Movement();
    headMovement();
  }

  telemetry.setSensorMeasurement(1, digitalRead(tension));
  
  // Console feedback
  Serial.print("Channel 1 (lr):");
  Serial.print(lr); Serial.print("\t");
  Serial.print("Channel 2 (fb):");
  Serial.print(fb); Serial.print("\t");
  Serial.print("Channel 4 (sv):");
  Serial.print(sv); Serial.print("\t");
  Serial.print("Channel 5 (ts):");
  Serial.print(ts); Serial.print("\n");
  Serial.print("Tension:");
  Serial.print(digitalRead(tension)); Serial.print("\n");
  delay(100); // Here just to make the terminal 
  
}

double top (double a, double b)
{
  if (a > b)
  {
    return a;
  }
  return b;
}

void Movement ()
{
  if (fb > 1550) // Moves FORWARD. Bigger than positive threshold  
  {
    vl = vlf * ((fb - 1550)/(1990 - 1550));
    vr = vrf * ((fb - 1550)/(1990 - 1550));
    if (lr < 1450)
    {
      vl = top ((vl - (vlf * ((1450 - lr)/(1450 - 990)))), 0);
    }
    if (lr > 1550)
    {
      vr = top ((vr - (vrf * ((lr - 1550)/(1990 - 1550)))), 0);
    }
    motorL.setSpeed(vl);
    motorL.run(FORWARD);
    motorR.setSpeed(vr);
    motorR.run(FORWARD);
  } 
  else if (fb < 1450)  // Moves BACKWARD. Lower than positive threshold
  {
    vl = vlf * ((1450 - fb)/(1450 - 990));
    vr = vrf * ((1450 - fb)/(1450 - 990));
    if (lr < 1450)
    {
      vl = top ((vl - (vlf * ((1450 - lr)/(1450 - 990)))), 0);
    }
    if (lr > 1550)
    {
      vr = top ((vr - (vrf * ((lr - 1550)/(1990 - 1550)))), 0);
    }
    motorL.setSpeed(vl);
    motorL.run(BACKWARD);
    motorR.setSpeed(vr);
    motorR.run(BACKWARD);
  } 
  else  // Stops 
  { 
    if (ts < 1450)  // Normal mode
    { 
      digitalWrite(ledPin, 0);  // Set eyes to not glow
      vl = 0;
      vr = 0;
      motorL.setSpeed(vl);
      motorL.run(FORWARD);
      motorR.setSpeed(vr);
      motorR.run(FORWARD);
    }
    else  // Crazy Turnz
    {  
      digitalWrite(ledPin, 255);  // Set eyes to glow
      if (lr < 1450)
      {
        vl = vlf * ((1450 - lr)/(1450 - 990));
        vr = vrf * ((1450 - lr)/(1450 - 990));
        motorL.setSpeed(vl);
        motorL.run(BACKWARD);
        motorR.setSpeed(vr);
        motorR.run(FORWARD);
      } 
      else if (lr > 1550)
      {
        vl = vlf * ((lr - 1550)/(1990 - 1550));
        vr = vrf * ((lr - 1550)/(1990 - 1550));
        motorL.setSpeed(vl);
        motorL.run(FORWARD);
        motorR.setSpeed(vr);
        motorR.run(BACKWARD);
      } 
      else 
      {
        vl = 0;
        vr = 0;
        motorL.setSpeed(vl);
        motorL.run(FORWARD);
        motorR.setSpeed(vr);
        motorR.run(FORWARD);
      }
    }
  }
}

void headMovement()
{
  int aux;
  aux = (180.0*(sv - 990))/1000.0;
  aux = top(aux, 0);
  head.write(aux);
}

void voice()
{
  int aux;
  aux = bz - 990;
  aux = top(aux, 0);
  
  //tone(buzzer, 500); 
  //noTone(buzzer);   
}
