

void (* resetFunc) (void) = 0;

#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <DMXSerial.h>

// difine
#define OLED_RESET 4
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define home_switch 9     // Pin 9 connected to Home Switch (MicroSwitch)
#define DMXchip 13 // pin op de rs485 chip low==read HIGH==Write
#define EEpromaAddress 0 // EEprom Address
#define KeypadPin A0 // EEprom Address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
elapsedMillis printTime;         // print time

// setup stepper
const int dirPin = 2;   // Direction
const int stepPin = 3;  // Step
AccelStepper Stepper1(AccelStepper::DRIVER, stepPin, dirPin);

const int maxSpeedLimit = 1000;  // set this to the maximum speed you want to use.
int long maxPositionLimit = 1200; // max steps
const int maxAcceleration =1000;  // set this to the maximum Acceleration you want to use.

long initial_homing = -1;  // Used to Home Stepper at startup

int DMXAdress =  EEPROM.get( EEpromaAddress, DMXAdress );  // Actual DMX adress stored in eprom
int DMXdisplayAdress = EEPROM.get( EEpromaAddress, DMXAdress );  // dmx adress in display
int DMXStatus = 0;                      //  // variable for dmx status
int LastDMXStatus = 0;                  // variable to detect lat dmx status
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;

int NoDataTime = 1000;  //Time to detect dmx change

long currentRollAngle;
long previousRollAngle;
long Roll;
long RoundCount = 0;
int CW = 0;
int CCW = 0;
long MakeRotate=0;
int Stopt=0;
int StoptB=0;
long diff;

//=========================== Setup =========================
void setup() {

  delay(50);

  

 // setup serial and display
  //Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Address 0x3C for 128x32

  // Clear display buffer.
  display.clearDisplay();
  display.display();

  // Setup the pins 
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(home_switch, INPUT_PULLUP);
  pinMode(KeypadPin, INPUT_PULLUP);

  // setup stepper
  Stepper1.setMaxSpeed(maxSpeedLimit);
  Stepper1.setSpeed(maxSpeedLimit);
  Stepper1.setAcceleration(maxAcceleration);

  DMXSerial.init(DMXReceiver); // reciver dmx
  digitalWrite(DMXchip, LOW); // pull dmx chip low

  // setup extra
  HOME();
  DisplayData();

  
  
}


// ============================ Loop ====================================
void loop() {

   // make position


  

  

  // ++++++++++++++++ DMX Status  ++++++++++++++++++++

  //Calculate how long no data bucket was received to determene dmx status
  unsigned long lastPacket = DMXSerial.noDataSince();

  if (lastPacket < NoDataTime) {
    DMXStatus = 1;
  } else {
    DMXStatus = 0;
  }
  if (DMXStatus != LastDMXStatus) {
    DisplayData();
  }
  LastDMXStatus = DMXStatus;

  if (DMXdisplayAdress != DMXAdress) {
  display.setCursor (10, 14); // position the cursor
  display.setTextSize (1); // medium size font
  display.setTextColor (WHITE); // white is not default !
  display.print ("Adress Not Stored");
  display.display();
   
  }
  //--------------------------------------------------

 // ++++++++++++++++ use button ++++++++++++++++++++

 int Keypad = analogRead(KeypadPin);

  if (Keypad>120 && Keypad < 160) {// menu
   delay(100);
  }

  if (Keypad >200 && Keypad < 300) { // enter
    DMXAdress = DMXdisplayAdress;
    EEPROM.put(EEpromaAddress, DMXAdress);
    DisplayData();
     delay(100);
  }
 if (Keypad >400 && Keypad < 460) {

    if ( DMXdisplayAdress==506){
       DMXdisplayAdress=0;
    }
    DMXdisplayAdress++;
    DisplayData();
     delay(50);
  }

  if (Keypad >1000 && Keypad < 1030) {

    if ( DMXdisplayAdress==1){
       DMXdisplayAdress=507;
    }
    DMXdisplayAdress--;
    DisplayData();
     delay(50);
  }

  //============


  // Read DMX CH1 16 bit 
  uint16_t CH1 = (DMXSerial.read(DMXAdress) << 8) | DMXSerial.read(DMXAdress + 1);


  // DMX To data
  int MotorPos = map(CH1, 0, 65535, 0, maxPositionLimit);
  int motorSpeed = map( DMXSerial.read(DMXAdress + 2), 0, 255, 0, maxSpeedLimit);
  int motorCW = DMXSerial.read(DMXAdress + 3);
  int CWExtra = map(motorCW,100,255,0,550);
  int motorCCW = DMXSerial.read(DMXAdress + 4);
  int CCWExtra = map(motorCCW,100,255,0,550);
  int ResetPos = DMXSerial.read(DMXAdress + 5);
  int ResetViaDMX = DMXSerial.read(DMXAdress + 6);
  
  

  //Reset
   if (ResetViaDMX > 128 ) {
    resetFunc();
  }

  // reset pos
 if(ResetPos > 128) {
   HOME();
}


  // Clockwise
  if (Stopt == 0 && motorCW > 100 && CCW == 0 && Stepper1.isRunning()==0) {
    CW = 1;
  }
  // Counter Clockwise
  if (Stopt == 0 && motorCCW > 100 && CW == 0 && Stepper1.isRunning()==0) {
    CCW = 1;
  }


  
  // stop clockwise
  if (motorCW < 100 && CW == 1 && Roll == (MotorPos) + ((RoundCount)*maxPositionLimit)) {
   
    CW = 0;
    CCW = 0;
    Stopt =1;
    
   
    
  }
  // stop counter clockwise
  if (motorCCW < 100 && CCW == 1 && Roll == (MotorPos) + ((RoundCount)*maxPositionLimit)) {
   
    CW = 0;
    CCW = 0;
    Stopt =1;
    
   
    
    
  }

  
 // make clockwise
  if (Stopt == 0 && CW == 1) {
    MakeRotate = Stepper1.currentPosition() - (MotorPos - 1) + 550;
  }
  // make counter clockwise
  if (Stopt == 0 && CCW == 1) {
    MakeRotate = Stepper1.currentPosition() - (MotorPos + 1) - 550;
  }

  

  
 Roll = MotorPos + MakeRotate;

  

  while(  Stepper1.isRunning()==0 && Stopt == 1){
    MakeRotate=0;
    previousRollAngle=Roll;
    RoundCount=0;
    Roll=MotorPos;
    previousRollAngle=Roll;
    currentRollAngle=Roll;

    Stepper1.setCurrentPosition(MotorPos);               // 0 sets the motor position to the 0° value as the zero position
    Stepper1.moveTo(Stepper1.currentPosition());  //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    Stepper1.setSpeed(10);
    Stepper1.run();
    Stopt=0;

    
    
    

   
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
   if (currentRollAngle - previousRollAngle < (-maxPositionLimit / 2) && currentRollAngle < previousRollAngle) {  //from 359 to 1



    // Stepper1.move(335);    // step x-amount to the right  to overcome the 'bridge', not used at the moment
    Stepper1.setCurrentPosition(0);               // 0 sets the motor position to the 0° value as the zero position
    Stepper1.moveTo(Stepper1.currentPosition());  //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    Stepper1.setSpeed(motorSpeed);
    Stepper1.run();


  }

  // if the current angle is e.g 359° and the previous angle is 1°, substracting them would give 358. So bigger than 180  AND if the current angle (359°) is bigger than the previous angle (1°):
  // ============================================================================================================================================================================================
  else if (currentRollAngle - previousRollAngle > (maxPositionLimit / 2) && currentRollAngle > previousRollAngle) {

    //Stepper1.move(-335);              // step x-amount to the left to overcome the 'bridge',not used at the moment
    Stepper1.setCurrentPosition(maxPositionLimit);      // 24120 sets the motor position to the 360° value as the zero position
    Stepper1.moveTo(Stepper1.currentPosition());  //motor still has to move to the '0' position as setCurrentPosition doesn't actually move the stepper
    Stepper1.setSpeed(motorSpeed);
    Stepper1.run();
  }
  Stepper1.run();


  // make previous angle
  previousRollAngle = currentRollAngle;  //  move the current reading to previous

  // no change during stop

  

  // make Roundcounter count
  RoundCount = (MakeRotate)/ maxPositionLimit;


  

}



//___________________________ DisplayData ________________________________________________________
void DisplayData() {
  // Clear the buffer
  display.clearDisplay();
  display.display();  // actually display all of the above

  // Draw a single pixel in white
  display.setCursor(10, 5);     // position the cursor
  display.setTextSize(1);       // medium size font
  display.setTextColor(WHITE);  // white is not default !
  display.print("DMX ADDRESS:");

  display.setCursor(80, 5);
  display.setTextSize(1);  // smallest font
  display.print(DMXdisplayAdress);

  if (DMXStatus == 1) {
    display.setCursor(90, 25);    // position the cursor
    display.setTextSize(1);       // medium size font
    display.setTextColor(WHITE);  // white is not default !
    display.print("DMX OK");
  } else {
    display.setCursor(90, 25);    // position the cursor
    display.setTextSize(1);       // medium size font
    display.setTextColor(WHITE);  // white is not default !
    display.print("NO DMX");
  }

  display.display();  // actually display all of the above
  Stepper1.setSpeed(maxSpeedLimit);
}


//___________________________ home ________________________________________________________
void HOME() {

  delay(500);

  display.clearDisplay();
  display.display(); //


  display.setCursor (10, 5); // position the cursor
  display.setTextSize (1); // medium size font
  display.setTextColor (WHITE); // white is not default !
  display.print ("Motor Is Going Home  ");

  display.display(); // actually display all of the above

  // Start Homing procedure of Stepper Motor at startup


  while (digitalRead(home_switch)) {  // Make the Stepper move CCW until the switch is activated   
    Stepper1.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    Stepper1.run();  // Start moving the stepper
    delay(20);
}

  Stepper1.setCurrentPosition(0);  // Set the current position as zero for now
  Stepper1.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  Stepper1.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing=1;

  while (!digitalRead(home_switch)) { // Make the Stepper move CW until the switch is deactivated
    Stepper1.moveTo(initial_homing);  
    Stepper1.run();
    initial_homing++;
    delay(20);
  }
  
  Stepper1.setCurrentPosition(0);

  display.clearDisplay();
  display.display(); //


  display.setCursor (10, 5); // position the cursor
  display.setTextSize (1); // medium size font
  display.setTextColor (WHITE); // white is not default !
  display.print ("Homing Completed");
  display.setCursor (10, 14); // position the cursor
  display.setTextSize (1); // medium size font
  display.setTextColor (WHITE); // white is not default !
  display.print ("maxPositionLimit");
  display.setCursor (10, 25); // position the cursor
  display.setTextSize (1); // medium size font
  display.setTextColor (WHITE); // white is not default !
  display.print (maxPositionLimit);

  display.display(); // actually display all of the above

  Stepper1.setMaxSpeed(maxSpeedLimit);
  Stepper1.setSpeed(maxSpeedLimit);
  Stepper1.setAcceleration(maxAcceleration);

  delay(2000);

  


}