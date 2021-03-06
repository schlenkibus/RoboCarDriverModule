#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
unsigned int sensors[5]; // an array to hold sensor values
unsigned int last_proportional = 0;
long integral = 0;



//Tuning Section
const int maximum = 70; //Maxspeed
const int maximumTurn = 60; //Maxspeed
auto timeToTurn = 350; //Zeit die für eine 90 Grad drehung benötigt wird
//End of Tuning Section


// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

void calibrateSensors() {
  for (auto counter=0; counter<83; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);
    robot.calibrateLineSensors(IR_EMITTERS_ON);
    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);
}


void setup()
{
  //Robo Init
  robot.init(2000);
  delay(10);
  OrangutanMotors::setSpeeds(0, 0);
  //End of Robo Init

  Serial.begin(115200);

  //Sensor Init
  OrangutanLCD::clear();
  OrangutanLCD::print("Press B");
  beep();
  OrangutanPushbuttons::waitForPress(BUTTON_B);
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);
  calibrateSensors();
  //End of Init
}

enum Instruktionen {
  F, S, L, R, T, B, E
};

char readUart(){
  int b;
  do {
    b = Serial.read();
  } while(b == -1);
  return static_cast<char>(b);
}

class foo   {
  public:
  foo() {
    
  }
  ~foo() {
    roboReady();
  }
};

void loop()
{
  char instruction;
  do {
    instruction = readUart();
    foo x;
    switch(instruction) {
      case 'F':
      moveB();
      goForward();
      break;
    case 'S':
       goForwardPlusStop();
      break;
    case 'L':
      moveB();
      turnLeft();
      break;
    case 'R':
      moveB();
      turnRight();
      break;
    case 'B':
      beep();
      break;
    case 'E':
      ende();
      break;
    case 'T':
      turn();
      break;
    
    }
  } while(true);

  exit(0);
  
  //Uart lesen und Instruktion ausführen
  auto i = readUart();
  //if (received_byte > 0) {
  switch(i) {
    
  }
   roboReady();
 //}
}

void goForward() {
  while(!isBreakCondition()) {
  //
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

  // The "proportional" term should be 0 when we are on the line.
  int proportional = (int)position - 2000;

  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - last_proportional;
  integral += proportional;

  // Remember the last position.
  last_proportional = proportional;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.  You can adjust the constants by which
  // the proportional, integral, and derivative terms are multiplied to
  // improve performance.
  int power_difference = proportional/20 + integral/10000 + derivative*3/2;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  if (power_difference > maximum){
    power_difference = maximum;
  }
  if (power_difference < -maximum){
     power_difference = -maximum;
  }


  if (power_difference < 0){
    OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
  }else{
    OrangutanMotors::setSpeeds(maximum, maximum - power_difference);
  }
  }
  OrangutanMotors::setSpeeds(0, 0); //Stoppen und auf neuen Befehl warten
}

void goForwardPlusStop(){
  OrangutanMotors::setSpeeds(maximum, maximum);
  delay(500); //Drive a half second forward
  OrangutanMotors::setSpeeds(0, 0);  //Stop at Sign
  beep();
  delay(50);//Zum Stillstand kommen
  goForward(); //Weiterfahren
}


void beep() {
  OrangutanBuzzer::playFrequency(6000, 100, 15); //6khz, 1s, max Lautstärke
  return;
}

void turnLeft(){
  OrangutanMotors::setSpeeds(-(maximumTurn), maximumTurn);
  delay(timeToTurn);
  OrangutanMotors::setSpeeds(0, 0);
  return;   
}

void turnRight(){
  OrangutanMotors::setSpeeds(maximumTurn, -(maximumTurn));
  delay(timeToTurn);
  OrangutanMotors::setSpeeds(0, 0);
  return;  
}

void turn(){
  turnRight();
  turnRight();
  return;
}

void ende(){
  beep();
  while(true);
  return;
}

void roboReady(){

  for(int i = 0; i<=20; i++){
  Serial.print('D');
  }
}

bool isBreakCondition(){
 int position = robot.readLine(sensors);
 
  // if all three front sensors see very low reflectance, take some appropriate action for this 
  // situation.
  if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
  {
    return true;
  }

    //Kreuzung
    else if(sensors[0] > 150 || sensors[4] > 150)
 {
    return true;
  } 
  return false;
}

void moveB(){
  OrangutanMotors::setSpeeds(50, 50);
  delay(315);
  OrangutanMotors::setSpeeds(0, 0);
}

