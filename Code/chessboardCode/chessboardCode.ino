#include <AccelStepper.h>
#include <MultiStepper.h>

// ===============================================================================
// PIN NUMBERING

#define limitSwitchY 0
#define limitSwitchX 1
#define magnetPin 6

// ===============================================================================
// BOARD CONSTANTS (all dims in mm)

const float boardSize = 457;
const float boardBoundryTop = 5;
const float boardBoundryBottom = 5;
const float boardBoundryLeft = 30;
const float boardBoundryRight = 10;

const float squareSize = boardSize / 8;

// ===============================================================================
// MOTORS

// Create the stepper objects
AccelStepper motA(1, 10, 11);
AccelStepper motB(1, 12, 13);
MultiStepper motPair;

#define stepsPerRev 200
#define microstepFactor 4
const float stepsPerMM = 20.3; // Only applies at quarter-stepping!!

#define VELOCITY_LOW 450
#define ACCEL_LOW 150
#define VELOCITY_HIGH 900
#define ACCEL_HIGH 300

// ===============================================================================
// SETUP

void setup()
{
  Serial.begin(9600);
  
  motPair.addStepper(motA);
  motPair.addStepper(motB);

  pinMode(limitSwitchX, INPUT_PULLUP);
  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(magnetPin, OUTPUT);
}

// ===============================================================================
// MAIN LOOP

void loop()
{
  Serial.println("Starting main loop...");
  delay(2000);
  calibrate();
  // moveRect(0, 150);
  delay(10000);
}

// ===============================================================================
// CALIBRATION

void calibrate()
{
  // Run at a slow speed
  motorSetLow();

  Serial.println("Current LimitSwitch Readings:");
  Serial.println(digitalRead(limitSwitchY));
  Serial.println(digitalRead(limitSwitchX));

  motA.move(-12000);
  motB.move(12000);
  
  while (digitalRead(limitSwitchY) == HIGH)
  {
    motA.run();
    motB.run();
  }
  Serial.println("Hard stop for Y-axis");
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);
  delay(250);
  
  motA.move(12000);
  motB.move(12000);
  
  while (digitalRead(limitSwitchX) == HIGH)
  {
    motA.run();
    motB.run();
  }
  Serial.println("Hard stop for X-axis");
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);
  delay(250);

  Serial.println("Moving to 1A...");
  moveRect(-1*(boardBoundryRight + boardSize), boardBoundryBottom);

  // Zero the final position as the bottom left of the 1A square
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);

  Serial.println("Calibration Complete!!");
}

void moveRect(int squaresX, int squaresY)
{
  // Run at full speed
  motorSetHigh();

  long stepsX = squaresToSteps(squaresX);
  long stepsY = squaresToSteps(squaresY);
  
  long steps[] = {-1*(stepsX + stepsY), (stepsX - stepsY)};
  motPair.moveTo(steps);
  motPair.runSpeedToPosition();
}

void moveDiag(int distanceX, int distanceY)
{
  // Run at full speed
  motorSetHigh();
}

void moveSmart(int startCoord, int endCoord)
{
  
}

void motorSetLow()
{
  Serial.println("Set motor speed/accel to LOW");
  motA.setMaxSpeed(VELOCITY_LOW);
  motA.setAcceleration(ACCEL_LOW);
  motB.setMaxSpeed(VELOCITY_LOW);
  motB.setAcceleration(ACCEL_LOW);
}

void motorSetHigh()
{
  Serial.println("Set motor speed/accel to HIGH");
  motA.setMaxSpeed(VELOCITY_HIGH);
  motA.setAcceleration(ACCEL_HIGH);
  motB.setMaxSpeed(VELOCITY_HIGH);
  motB.setAcceleration(ACCEL_HIGH);
}

long squaresToSteps(int squares)
{
  long distanceMM = squares * squareSize;
  long steps = distanceMM * stepsPerMM;
  return steps;
}
