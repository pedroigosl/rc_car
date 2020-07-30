/*
 * Project: Arduino RC Car with iBUS Receiver and iBUS battery tension feedback
 * Author: Pedro Igo Sousa Lucas
 */

#include <Servo.h>
#include <AFMotor.h>  // By Adafruit (Access Mar 10th 2020 https://github.com/adafruit/Adafruit-Motor-Shield-library)

#include <IBusBM.h>   // By bmellink (Access Jul 30th 2020 https://github.com/bmellink/IBusBM)
  
#include <iBUSTelemetry.h>    // By adis1313 (Access Jul 30th https://github.com/adis1313/iBUSTelemetry-Arduino)

/*
 * OBS1: IBusBM.h also has telemetry, but requires hardware mods (including a registor), so used another for telemetry reading
 * OBS2: Originally used telemetry by Hrastovc (Access Mar 10th 2020 https://github.com/Hrastovc/iBUStelemetry), but changed 
 * over due to incompatibilities with IBusBM. New telemetry library has more sensors with more data types, but is a bit fussier 
 * with delays. Still works fine.
 */

#define channelAmount 5 // Number of channels

/******************************************
            GLOBAL VARIABLES
******************************************/

// Motors declaration. AFMotor library
AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

double inputChannel [channelAmount];

double vl, vr; // Motor velocities
// Motor Factors. Adjust to compensate for weaker motors. Max = 200, Min = 0
// setSpeed can receive 0 - 255, but i limited to 200 to avoid motor strain
// OBS: Only necessary if your controller doesn't have trim
double vlf = 250;
double vrf = 250;

int ledPin = 54; // Eye led's pin
int buzzer = 18;
int servoPin = 9; 

// Variables for calculating tension
int tensionSensor = A1; // Reading from voltage sensor
float tension; // Calulated tension
double r1 = 30000; // Voltage sensor resistor 1
double r2 = 7500; // Voltage sensor resistor 2

Servo head;

iBUSTelemetry telemetry(50); // Sets telemetry output pin. On my mega, only worked on SPI pins

IBusBM IBus;    // Declares IBus object for Rx inputs

// Channels
double lr; // Left / Right
double fb; // Forward / Back
// double bz; // Buzzer
double sv; // Servo
double ts; // Turn style

/******************************************
               MAIN PROGRAM
******************************************/

//Receiver input assignment
void setup() 
{
  pinMode (tensionSensor, INPUT);
  pinMode(ledPin, OUTPUT); //---pinMode(buzzer, OUTPUT);
  
  head.attach(servoPin);
  
  Serial.begin(115200); 

  // Data receiving
  IBus.begin(Serial1); // Needs to be associated to a serial port

   telemetry.begin(); // Sets telemetry

    telemetry.addSensor(IBUS_MEAS_TYPE_EXTV); // Adds an external voltage sensor to telemetry output
}

void loop() 
{
  // input receiving (iBus reading)
  // OBS: Pin A0 / 14 has weak solder and keeps losing contact
  telemetry.run(); // Starts telemetry reading
  for (int channel = 0; channel < channelAmount; ++channel)
  {
    inputChannel[channel] = IBus.readChannel(channel);
  }
  lr = inputChannel [0];
  fb = inputChannel [1];
//  bz = inputChannel [2];
  sv = inputChannel [3];
  ts = inputChannel [4];

  // voice();
  if (lr > 1)
  {
    Movement();
    headMovement();
  }

  // Calculates tension
  tension = printTension(analogRead(tensionSensor), r1, r2);
  telemetry.setSensorValueFP(1, tension); // Sets float value to telemetry
  
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
  Serial.print(tension); Serial.print("\n");
  
}

/******************************************
                FUNCTIONS
******************************************/

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

// Calculates tension reading
double printTension(int reading, double resistor_1, double resistor_2)
{
  return ((reading * 5)/((1024 * resistor_2)/(resistor_1 + resistor_2)));
}

/*
void voice()
{
  int aux;
  aux = bz - 990;
  aux = top(aux, 0);
  
  //tone(buzzer, 500); 
  //noTone(buzzer);   
}
*/
