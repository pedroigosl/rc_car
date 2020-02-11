/* 
 * Project: Infra-red arduino remote controlled car
 * Autor: Pedro Igo Sousa Lucas
*/

#include <AFMotor.h>
//#include <Servo.h> 
#include <IRremote.h>  

AF_DCMotor motorL(1);
AF_DCMotor motorR(2);

//Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards

const int irReceiverPin = A0;
IRrecv irrecv(irReceiverPin); 
decode_results results;   

int n, ser, vr, vl; 

void setup() {
  
//  myservo.attach(10);
  n = 0;

  //Serial.begin(9600);   
  irrecv.enableIRIn();   
}

void loop() {

    if (irrecv.decode(&results)) {   
      //Serial.print("irCode: ");            
      //Serial.print(results.value, DEC);
      switch (results.value){
        case 16718055: //2
          n = 2;
          break; 
        case 16716015: //4
          n = 4;
          break; 
        case 16734885: //6
          n = 6;
          break; 
        case 16730805: //8
          n = 8;
          break; 
        case 16724175: //1
          n = 1;
          break; 
        case 16743045: //3
          n = 3;
          break;
        case 16750695: //crazy
          n = 9;
          break;  
        default:
          n = 0;
          break;
      }
    //  Serial.print(",  bits: ");           
    //  Serial.println(results.bits); 
      irrecv.resume();    
    } 
    
    switch (n){
      case 0:
        //ser = 1500;
        vr = 0;
        vl = vr;
        runFor();
        break;
      case 2:
        //ser = 1500;
        vr = 200;
        vl = vr;
        runFor();
        break;
      case 4:
        //ser = 1500;
        vr = 200;
        vl = 0;
        runFor();
        break;
      case 6:
        //ser = 1500;
        vr = 0;
        vl = 200;
        runFor();
        break;
      case 8:
        //ser = 1500;
        vr = 200;
        vl = vr;
        runBack();
        break;
      case 1:
        //ser = 1500;
        vr = 200;
        vl = 100;
        runFor();
        break;
      case 3:
        //ser = 1500;
        vr = 100;
        vl = 200;
        runFor();
        break;
      case 9:
        //ser = 1500;
        vr = 200;
        vl = 200;
        runDevil();
        break;
    }
    delay(200);
    //myservo.writeMicroseconds(ser);
}

void runFor(){
    motorL.setSpeed(vl);
    motorL.run(BACKWARD);
    motorR.setSpeed(vr);
    motorR.run(BACKWARD);
}

void runBack(){
    motorL.setSpeed(vl);
    motorL.run(FORWARD);
    motorR.setSpeed(vr);
    motorR.run(FORWARD);
}

void runDevil(){
    motorL.setSpeed(vl);
    motorL.run(BACKWARD);
    motorR.setSpeed(vr);
    motorR.run(FORWARD);
}



