
#include <DMXSerial.h>
#include <AccelStepper.h>
#include <elapsedMillis.h>
#define DMXchip 13 // pin op de rs485 chip low==read HIGH==Write

elapsedMillis RoundTime; 
elapsedMillis printTime; 



const int dirPin = 2;                                          // Direction
const int stepPin = 3;                                         // Step
AccelStepper RollStepperA(AccelStepper::DRIVER, stepPin, dirPin);

int DMXAdress = 1; // dmx start adress

long currentRollAngle;
long previousRollAngle;
long Roll;
long RoundCount = 0 ;
int stepsRound= 4805; //stappen voor vol rondje
int MotorRotate = 0;
long extra=1;
int CW=0;
int Stop=0;
long ConMotorRotate2;



void setup() {
  Serial.begin(115200);
    
    // setup dmx
  DMXSerial.init(DMXReceiver); // reciver dmx
  digitalWrite(DMXchip, LOW); // pull dmx chip low

  // setup stepper
  RollStepperA.setMaxSpeed(4000);
  RollStepperA.setSpeed(2000);
  RollStepperA.setAcceleration(1000);
  RollStepperA.setCurrentPosition(0);
  
}

void loop() {

////////////////////////// 
//Moet dit in loop? DMX adres vastzetten bij opstart is ook prima toch? scheelt heel veel writes die toch niets doen
/////////////////////////
uint16_t CH1 = (DMXSerial.read(DMXAdress)<<8) | DMXSerial.read(DMXAdress + 1); 
int CH3 = DMXSerial.read(DMXAdress + 2);
int CH4 = DMXSerial.read(DMXAdress + 3);
int CH5 = DMXSerial.read(DMXAdress + 4);
int CH6 = DMXSerial.read(DMXAdress + 5);



int motorPos = map(CH1, 0, 65535, 0, stepsRound);
int motorSpeed = map(CH3, 0, 255, 1, 4000);
int motorAccel = map(CH4, 0, 255, 0, 4000);
int motorCW = CH5;
int motorCCW = CH6;

     




if (CH5>100  ){
  CW=1;
   }
 
 Roll = motorPos + ConMotorRotate2 ;


if ( CH5<100 && CW==1 && Stop==0 &&  Roll==motorPos + ((RoundCount+1)*stepsRound) ){
  Stop=1;
  Serial.print("stop----------------------------------------------------------");
  Serial.println(RollStepperA.currentPosition());
  Serial.print("Target=Roll. ");
  Serial.println(Roll);
  Serial.print("motorPos ");
  Serial.println(motorPos);
  Serial.print("RoundCount  ");
  Serial.println(RoundCount);
  Serial.print("Rontje ");
  Serial.println(stepsRound);
  Serial.print("RoundCount*stepsRound ");
  Serial.println(RoundCount*stepsRound);
   
}

if(CW==1&& Stop==0){ 
      ConMotorRotate2 = RollStepperA.currentPosition()-(motorPos-1)+stepsRound/2  ;
      
   }



/////////////////
//Hier snap ik het even niet:
//Je zet hier al een beweging in, maar daarna check je nog of die over 360/0 graden positie gaat en doe je die vertalling en run.
//Het lijkt mij dat dit conflicten geeft, of dat het bridging 0-360 script nooit gebruikt word, want schijf verplaatst
///////////////
//Daarnaast set je hier de setMaxSpeed, ipv setSpeed. Dit lijkt mij niet de bedoeling??
/////////////////
  currentRollAngle = Roll;
   
  RollStepperA.setMaxSpeed(motorSpeed); //
  RollStepperA.moveTo(Roll );
  RollStepperA.run();


  //================bridging 0 - 360 ======================
  // if the current angle is e.g 1° and the previous angle is 359, substracting them would give -358. So smaller than -180  AND if the current angle (1°) is smaller than the previous angle (359°):
  // ============================================================================================================================================================================================
  if (currentRollAngle - previousRollAngle < (-stepsRound/2) && currentRollAngle < previousRollAngle) {  //from 359 to 1



    // RollStepperA.move(335);    // step x-amount to the right  to overcome the 'bridge', not used at the moment
    RollStepperA.setCurrentPosition(0);         // 0 sets the motor position to the 0° value as the zero position
    RollStepperA.moveTo(RollStepperA.currentPosition()); //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    RollStepperA.setSpeed(motorSpeed);
    RollStepperA.run();

    
  }

  // if the current angle is e.g 359° and the previous angle is 1°, substracting them would give 358. So bigger than 180  AND if the current angle (359°) is bigger than the previous angle (1°):
  // ============================================================================================================================================================================================
  else if (currentRollAngle - previousRollAngle > (stepsRound/2) && currentRollAngle > previousRollAngle) {

    //RollStepperA.move(-335);              // step x-amount to the left to overcome the 'bridge',not used at the moment
    RollStepperA.setCurrentPosition(stepsRound); // 24120 sets the motor position to the 360° value as the zero position
    RollStepperA.moveTo(RollStepperA.currentPosition()); //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    RollStepperA.setSpeed(motorSpeed);
    RollStepperA.run();

   
  }
  RollStepperA.run();

  previousRollAngle = currentRollAngle;  //  move the current reading to previous

  RoundCount = RollStepperA.currentPosition()/stepsRound;
 if (printTime ==1000){
   printTime =0;
Serial.println(RollStepperA.currentPosition());
//Serial.println(RoundCount);

 }

}
