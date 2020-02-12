/*
* Project: Arduino RC Car with PWM Receiver
* Autor: Pedro Igo Sousa Lucas
*/

#include <AFMotor.h>

#define ledPin 14 // Eye led's pin

// Motors declaration. AFMotor library
AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

double vl, vr; // Motor velocities
// Motor Factors. Adjust to compensate for weaker motors. Max = 200, Min = 0
// setSpeed can receive 0 - 255, but i limited to 200 to avoid motor strain
// OBS: Only necessary if your controller doesn't have trim
double vlf = 190;
double vrf = 200;

//Receiver channels
double lr; // Left / Right
double fb; // Forward / Back
double ts; // Turn style

//Receiver input assignment
void setup() 
{
  pinMode(14, OUTPUT); pinMode(17, INPUT); pinMode(18, INPUT); pinMode(19, INPUT);
  Serial.begin(9600); 
}



void loop() 
{
  // input receiving (PWM reading)
  // OBS: Pin A0 / 14 has weak solder and keeps losing contact
  lr = pulseIn(17, HIGH);
  fb = pulseIn(18, HIGH);
  ts = pulseIn(19, HIGH);

  if (lr > 1)
  {
    Movement();
  }
  
  // Console feedback
  Serial.print("Channel 1 (lr):");
  Serial.print(lr); Serial.print("\t");
  Serial.print("Channel 2 (fb):");
  Serial.print(fb); Serial.print("\t");
  Serial.print("Channel 5 (ts):");
  Serial.print(ts); Serial.print("\n");
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
      analogWrite(ledPin, 0);  // Set eyes to not glow
      vl = 0;
      vr = 0;
      motorL.setSpeed(vl);
      motorL.run(FORWARD);
      motorR.setSpeed(vr);
      motorR.run(FORWARD);
    }
    else  // Crazy Turnz
    {  
      analogWrite(ledPin, 255);  // Set eyes to glow
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
