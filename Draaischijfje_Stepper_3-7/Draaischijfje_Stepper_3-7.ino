#include <DMXSerial.h>
#include <AccelStepper.h>
#include <elapsedMillis.h>
#define DMXchip 13  // pin op de rs485 chip low==read HIGH==Write

elapsedMillis printTime;

// setup stepper
const int dirPin = 2;   // Direction
const int stepPin = 3;  // Step
AccelStepper Stepper1(AccelStepper::DRIVER, stepPin, dirPin);

// set dmx adress
int DMXAdress = 1;  // dmx start adress

long currentRollAngle;
long previousRollAngle;
long Roll;
long RoundCount = 0;
int StepsRound = 4805;
int CW = 0;
int CCW = 0;
int Stop =0;
long MakeRotate;

//=========================== Setup =========================
void setup() {
  Serial.begin(115200);

  // setup dmx
  DMXSerial.init(DMXReceiver);  // reciver dmx
  digitalWrite(DMXchip, LOW);   // pull dmx chip low

  // setup stepper
  Stepper1.setMaxSpeed(4000);
  Stepper1.setSpeed(2000);
  Stepper1.setAcceleration(1000);
  Stepper1.setCurrentPosition(0);
}


// ============================ Loop ====================================
void loop() {

  // Read DMX
  uint16_t CH1 = (DMXSerial.read(DMXAdress) << 8) | DMXSerial.read(DMXAdress + 1);
  int CH3 = DMXSerial.read(DMXAdress + 2);
  int CH4 = DMXSerial.read(DMXAdress + 3);
  int CH5 = DMXSerial.read(DMXAdress + 4);
  int CH6 = DMXSerial.read(DMXAdress + 5);


  // DMX To data
  int MotorPos = map(CH1, 0, 65535, 0, StepsRound);
  int motorSpeed = map(CH3, 0, 255, 1, 4000);
  int motorAccel = map(CH4, 0, 255, 0, 4000);
  int motorCW = CH5;
  int motorCCW = CH6;


  // Clockwise
  if (motorCW > 100 && CCW == 0 && Stepper1.isRunning()==0) {
    CW = 1;
  }
  // Counter Clockwise
  if (motorCCW > 100 && CW == 0 && Stepper1.isRunning()==0) {
    CCW = 1;
  }


  // make position
  Roll = MotorPos + MakeRotate;

  // stop clockwise
  if (motorCW < 100 && CW == 1 && Roll == MotorPos + ((RoundCount)*StepsRound)) {
    CW = 0;
    CCW = 0;
    // Serial.print("stop----------------------------------------------------------");
    // Serial.println(Stepper1.currentPosition());
    // Serial.print("Target=Roll. ");
    // Serial.println(Roll);
    // Serial.print("MotorPos ");
    // Serial.println(MotorPos);
    // Serial.print("RoundCount  ");
    // Serial.println(RoundCount);
    // Serial.print("StepsRound ");
    // Serial.println(StepsRound);
    // Serial.print("RoundCount*StepsRound ");
    // Serial.println(RoundCount*StepsRound);
  }
  // stop counter clockwise
  if (motorCCW < 100 && CCW == 1 && Roll == MotorPos + ((RoundCount)*StepsRound)) {
    CCW = 0;
  }


  // make clockwise
  if (CW == 1) {
    MakeRotate = Stepper1.currentPosition() - (MotorPos - 1) + 2000;
  }
  // make counter clockwise
  if (CCW == 1) {
    MakeRotate = Stepper1.currentPosition() - (MotorPos - 1) - 2000;
  }

  // make current Angle
  currentRollAngle = Roll;


  // make motor move
  Stepper1.setMaxSpeed(motorSpeed);
  Stepper1.moveTo(Roll);
  Stepper1.run();


  //================bridging 0 - 360 ======================
  // if the current angle is e.g 1° and the previous angle is 359, substracting them would give -358. So smaller than -180  AND if the current angle (1°) is smaller than the previous angle (359°):
  // ============================================================================================================================================================================================
  if (currentRollAngle - previousRollAngle < (-StepsRound / 2) && currentRollAngle < previousRollAngle) {  //from 359 to 1



    // Stepper1.move(335);    // step x-amount to the right  to overcome the 'bridge', not used at the moment
    Stepper1.setCurrentPosition(0);               // 0 sets the motor position to the 0° value as the zero position
    Stepper1.moveTo(Stepper1.currentPosition());  //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    Stepper1.setSpeed(motorSpeed);
    Stepper1.run();


  }

  // if the current angle is e.g 359° and the previous angle is 1°, substracting them would give 358. So bigger than 180  AND if the current angle (359°) is bigger than the previous angle (1°):
  // ============================================================================================================================================================================================
  else if (currentRollAngle - previousRollAngle > (StepsRound / 2) && currentRollAngle > previousRollAngle) {

    //Stepper1.move(-335);              // step x-amount to the left to overcome the 'bridge',not used at the moment
    Stepper1.setCurrentPosition(StepsRound);      // 24120 sets the motor position to the 360° value as the zero position
    Stepper1.moveTo(Stepper1.currentPosition());  //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    Stepper1.setSpeed(motorSpeed);
    Stepper1.run();
  }
  Stepper1.run();


  // make previous angle
  previousRollAngle = currentRollAngle;  //  move the current reading to previous

  // make Roundcounter count
  RoundCount = MakeRotate / StepsRound;

  // reset print timer and print
  if (printTime == 1000) {
    printTime = 0;
    Serial.println(Stepper1.isRunning());
  }

}
