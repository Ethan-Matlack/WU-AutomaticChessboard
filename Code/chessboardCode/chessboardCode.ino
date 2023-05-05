#include <AccelStepper.h>
#include <MultiStepper.h>

// ===============================================================================
// PIN NUMBERING

// Digitals
#define limitSwitchY 0
#define limitSwitchX 1
const byte MUX_SELECT [4] = {2, 3, 4, 5};
#define magnet 6

// Analogs
const byte MUX_SIGNAL [4] = {A0, A1, A2, A3};

// ===============================================================================
// BOARD CONSTANTS (all dims in mm)

const int boardSize = 457;
const int boardBoundryTop = 5;
const int boardBoundryBottom = 5;
const int boardBoundryLeft = 30;
const int boardBoundryRight = 10;

const float squareSize = boardSize / 8;

// ===============================================================================
// MOTORS

// Create the stepper objects
AccelStepper motA(1, 10, 11);
AccelStepper motB(1, 12, 13);
MultiStepper motPair;

#define STEPS_PER_REV 200
#define MICROSTEP_FACTOR 4
const float stepsPerMM = 20.3; // Only applies at quarter-stepping!!

#define VELOCITY_LOW 450
#define ACCEL_LOW 150
#define VELOCITY_HIGH 900
#define ACCEL_HIGH 300

// ===============================================================================
// MULTIPLEXERS & REED SWITCHES

const byte MUX_CHANNEL[16][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 1},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 1, 0, 0},
  {0, 1, 0, 1},
  {0, 1, 1, 0},
  {0, 1, 1, 1},
  {1, 1, 1, 1},
  {1, 1, 1, 0},
  {1, 1, 0, 1},
  {1, 1, 0, 0},
  {1, 0, 1, 1},
  {1, 0, 1, 0},
  {1, 0, 0, 1},
  {1, 0, 0, 0},
};

int reed_sensor_status [8][8];

// ===============================================================================
// SETUP

void setup()
{
  // Start the serial console
  Serial.begin(9600);
  
  // Add the steppers to a group
  motPair.addStepper(motA);
  motPair.addStepper(motB);

  // Set all the pin modes
  pinMode(limitSwitchX, INPUT_PULLUP);
  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(MUX_SELECT[0], INPUT);
  pinMode(MUX_SELECT[1], INPUT);
  pinMode(MUX_SELECT[2], INPUT);
  pinMode(MUX_SELECT[3], INPUT);
  pinMode(magnet, OUTPUT);
  pinMode(MUX_SIGNAL[0], INPUT);
  pinMode(MUX_SIGNAL[1], INPUT);
  pinMode(MUX_SIGNAL[2], INPUT);
  pinMode(MUX_SIGNAL[3], INPUT);

  Serial.println("Starting main loop...");
}

// ===============================================================================
// MAIN LOOP

void loop()
{
  delay(5000);
  // calibrate();
  // moveRect(0, 150);
  readReedSwitches();
}

// ===============================================================================
// CALIBRATION

void calibrate()
{
  // Run at a slow speed
  motorSetLow();

  // Set a large goal for zero-ing the Y-axis
  motA.move(-12000);
  motB.move(12000);
  
  while (digitalRead(limitSwitchY) == HIGH)
  {
    motA.run();
    motB.run();
  }
  Serial.println("Limit reached for Y-axis");
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);
  delay(250);
  
  // Set a large goal for zero-ing the X-axis
  motA.move(12000);
  motB.move(12000);
  
  while (digitalRead(limitSwitchX) == HIGH)
  {
    motA.run();
    motB.run();
  }
  Serial.println("Limited reached for X-axis");
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);
  delay(250);

  // Move to the lower left edge of the A1 square
  moveRect(-1*(boardBoundryRight + boardSize), boardBoundryBottom);

  // Zero the final position
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);

  Serial.println("Calibration Complete!!");
}

// ===============================================================================
// MOVEMENT FUNCTIONS

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

// ===============================================================================
// MAGNET CONTROL

void electromagnet(bool state)
{
  if (state == true) {
    digitalWrite(magnet, HIGH);
    delay(600);
  }
  else {
    delay(600);
    digitalWrite(magnet, LOW);
  }
}

// ===============================================================================
// MULTIPLEXER READING

void readReedSwitches()
{
  Serial.println("Fetching new values...");

  // Read the values and store to an array
  for (byte i = 0; i < 16; i++) {
    // Set the select pins based on the table
    digitalWrite(MUX_SELECT[0], MUX_CHANNEL[i][0]);
    digitalWrite(MUX_SELECT[1], MUX_CHANNEL[i][1]);
    digitalWrite(MUX_SELECT[2], MUX_CHANNEL[i][2]);
    digitalWrite(MUX_SELECT[3], MUX_CHANNEL[i][3]);
    delay(1);

    // If we're looking at the first column of the bank...
    if (i < 8) {
      reed_sensor_status[i][0] = digitalRead(MUX_SIGNAL[0]);
      reed_sensor_status[i][2] = digitalRead(MUX_SIGNAL[1]);
      reed_sensor_status[i][4] = digitalRead(MUX_SIGNAL[2]);
      reed_sensor_status[i][6] = digitalRead(MUX_SIGNAL[3]);
    }

    // Otherwise, we must be looking at the second bank column...
    else {
      reed_sensor_status[i-8][1] = digitalRead(MUX_SIGNAL[0]);
      reed_sensor_status[i-8][3] = digitalRead(MUX_SIGNAL[1]);
      reed_sensor_status[i-8][5] = digitalRead(MUX_SIGNAL[2]);
      reed_sensor_status[i-8][7] = digitalRead(MUX_SIGNAL[3]);
    }
  }
  delay(100);

  for (byte i = 0; i < 8; i++) {
    Serial.print(reed_sensor_status[i][0]);
    Serial.print(reed_sensor_status[i][1]);
    Serial.print(reed_sensor_status[i][2]);
    Serial.print(reed_sensor_status[i][3]);
    Serial.print(reed_sensor_status[i][4]);
    Serial.print(reed_sensor_status[i][5]);
    Serial.print(reed_sensor_status[i][6]);
    Serial.print(reed_sensor_status[i][7]);
    Serial.println();
  }
}

// ===============================================================================
// MOTOR CONTROL PRESETS

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

// ===============================================================================
// CONVERSION FUNCTIONS

long squaresToSteps(int squares)
{
  long distanceMM = squares * squareSize;
  long steps = distanceMM * stepsPerMM;
  return steps;
}
