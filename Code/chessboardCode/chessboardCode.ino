#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Micro_Max.h"

// ===============================================================================
// PIN NUMBERING

// Digitals
#define limitSwitchY 0
#define limitSwitchX 1
const byte MUX_SELECT [4] = {2, 3, 4, 5};
#define magnet 6
#define turnComplete 7

// Analogs
const byte MUX_SIGNAL [4] = {A0, A1, A2, A3};

// ===============================================================================
// BOARD CONSTANTS

// All dimensions in mm
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

#define SPEED_SLOW 0
#define SPEED_FAST 1

byte trolley_coordinate_X = 0;
byte trolley_coordinate_Y = 0;

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
int reed_sensor_status_memory [8][8];


// ===============================================================================
// GAME MANAGEMENT

enum {calibration, player_white, player_black};
byte sequence = calibration;

// Move updates
char mov [4] = {0, 0, 0, 0};
byte stateUpdate_col[2];
byte stateUpdate_row[2];

// MicroMax
extern char lastH[], lastM[];

// ===============================================================================
// SETUP

void setup()
{
  // Start the serial console
  Serial.begin(9600);
  
  // Add the steppers to a group
  motPair.addStepper(motA);
  motPair.addStepper(motB);

  // Digital pin modes
  pinMode(limitSwitchX, INPUT_PULLUP);
  pinMode(limitSwitchY, INPUT_PULLUP);
  pinMode(MUX_SELECT[0], INPUT);
  pinMode(MUX_SELECT[1], INPUT);
  pinMode(MUX_SELECT[2], INPUT);
  pinMode(MUX_SELECT[3], INPUT);
  pinMode(magnet, OUTPUT);
  pinMode(turnComplete, INPUT_PULLUP);

  // Analog pin modes
  pinMode(MUX_SIGNAL[0], INPUT);
  pinMode(MUX_SIGNAL[1], INPUT);
  pinMode(MUX_SIGNAL[2], INPUT);
  pinMode(MUX_SIGNAL[3], INPUT);

  //  Set the reed switches status
  for (byte i = 2; i < 6; i++) {
    for (byte j = 0; j < 8; j++) {
      reed_sensor_status[i][j] = 1;
      reed_sensor_status_memory[i][j] = 1;
    }
  }

  //  MicroMax
  lastH[0] = 0;

  Serial.println("Starting main loop...");
}

// ===============================================================================
// MAIN LOOP

void loop()
{
  switch (sequence) {
    case calibration:
      calibrate();
      sequence = player_white;
      break;
    
    case player_white:
      waitForPlayerMove();
      decodePlayerMove();
      AI_HvsC();
      sequence = player_black;
      delay(500);
      break;

    case player_black:
      playComputerMove();
      sequence = player_white;
      delay(500);
      break;
  }
}

// ===============================================================================
// CALIBRATION

void calibrate()
{
  // Run at a slow speed
  motorsSetSpeed(SPEED_SLOW);

  // Set a large goal for zero-ing the Y-axis and move towards it until hitting the limit switch
  motA.move(-12000);
  motB.move(12000);
  while (digitalRead(limitSwitchY) == HIGH) motPair.run();
  Serial.println("Limit reached for Y-axis");
  zeroMotors();
  delay(250);
  
  // Set a large goal for zero-ing the Y-axis and move towards it until hitting the limit switch
  motA.move(12000);
  motB.move(12000);
  while (digitalRead(limitSwitchX) == HIGH) motPair.run();
  Serial.println("Limited reached for X-axis");
  zeroMotors();
  delay(250);

  // Move to the lower left edge of the A1 square and zero
  moveRectMM(-1*(boardBoundryRight + boardSize), boardBoundryBottom, SPEED_FAST);
  zeroMotors();

  Serial.println("Calibration Complete!!");
}

// ===============================================================================
// PLAYER MOVE

void waitForPlayerMove()
{
  Serial.println("Wait for board scan...");
  readReedSwitches();
  Serial.println("Make your move! Click the button when done.");

  // Infinite loop until the button is pressed
  while (digitalRead(turnComplete) == HIGH);
}

void decodePlayerMove()
{
  //  Compare the old and new status of the reed switches
  for (byte i = 0; i < 8; i++) {
    for (byte j = 0; j < 8; j++) {
      if (reed_sensor_status[i][j] != reed_sensor_status_memory[i][j]) {
        if (reed_sensor_status_memory[i][j] == 1) {
          stateUpdate_col[0] = i;
          stateUpdate_row[0] = j;
        }
        if (reed_sensor_status_memory[i][j] == 0) {
          stateUpdate_col[1] = i;
          stateUpdate_row[1] = j;
        }
      }
    }
  }

  //  Set the new status of the reed sensors
  for (byte i = 0; i < 8; i++) {
    for (byte j = 0; j < 8; j++) {
      reed_sensor_status[i][j] = reed_sensor_status_memory[i][j];
    }
  }

  //  Convert the reed sensors switches coordinates in characters
  char table1[] = {'8', '7', '6', '5', '4', '3', '2', '1'};
  char table2[] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};

  // Store the state change (move) as an array of chars
  mov[0] = table2[stateUpdate_row[0]];
  mov[1] = table1[stateUpdate_col[0]];
  mov[2] = table2[stateUpdate_row[1]];
  mov[3] = table1[stateUpdate_col[1]];
}

// ===============================================================================
// COMPUTER MOVE

void playComputerMove()
{
  //  Convert the AI characters in variables
  int departure_coord_X = lastM[0] - 'a' + 1;
  int departure_coord_Y = lastM[1] - '0';
  int arrival_coord_X = lastM[2] - 'a' + 1;
  int arrival_coord_Y = lastM[3] - '0';
  byte displacement_X = 0;
  byte displacement_Y = 0;

  //  Check if the move was a capture
  int convert_table [] = {0, 7, 6, 5, 4, 3, 2, 1, 0};
  byte white_capturing = 1;
  if (reed_sensor_status_memory[convert_table[arrival_coord_Y]][arrival_coord_X - 1] == 0) white_capturing = 0;

  // If the move was a capture, go ahead and get rid of that piece first
  for (byte i = white_capturing; i < 2; i++) {
    if (i == 0) {
      displacement_X = abs(arrival_coord_X - trolley_coordinate_X);
      displacement_Y = abs(arrival_coord_Y - trolley_coordinate_Y);
    }
    else if (i == 1) {
      displacement_X = abs(departure_coord_X - trolley_coordinate_X);
      displacement_Y = abs(departure_coord_Y - trolley_coordinate_Y);
    }
    
    // Traverse the trolley over to the piece that needs to be removed (if 1st iteration)
    // If on 2nd iteration, this will traverse to the player's piece that needs movement
    if (departure_coord_X > trolley_coordinate_X) moveRect(-displacement_X, 0, SPEED_SLOW);
    else if (departure_coord_X < trolley_coordinate_X) moveRect(displacement_X, 0, SPEED_SLOW);
    if (departure_coord_Y > trolley_coordinate_Y) moveRect(0, displacement_Y, SPEED_SLOW);
    else if (departure_coord_Y < trolley_coordinate_Y) moveRect(0, -displacement_Y, SPEED_SLOW);

    if (i == 0) {

      // Move the piece off the board
      electromagnet(true);
      moveRect(-0.5, 0, SPEED_SLOW);
      moveRect(0, arrival_coord_X - 0.5, SPEED_SLOW);
      electromagnet(false);

      // Go back
      moveRect(0.5, 0, SPEED_SLOW);
      moveRect(0, 0.5 - arrival_coord_X, SPEED_SLOW);

      // Reset the coords to current position
      trolley_coordinate_X = arrival_coord_X;
      trolley_coordinate_Y = arrival_coord_Y;
    }
  }

  // Reset the coords to current position
  trolley_coordinate_X = arrival_coord_X;
  trolley_coordinate_Y = arrival_coord_Y;

  // Calc the displacement to make the move
  displacement_X = arrival_coord_X - departure_coord_X;
  displacement_Y = arrival_coord_Y - departure_coord_Y;

  electromagnet(true);

  // Knight displacement
  if (displacement_X == 1 && displacement_Y == 2 || displacement_X == 2 && displacement_Y == 1) {
    if (displacement_Y == 2) {
      moveRect((displacement_X * 0.5), 0, SPEED_SLOW);
      moveRect(displacement_Y, 0, SPEED_SLOW);
      moveRect((displacement_X * 0.5), 0, SPEED_SLOW);
    }
    else if (displacement_X == 2) {
      moveRect(0, (displacement_Y * 0.5), SPEED_SLOW);
      moveRect(0, displacement_X, SPEED_SLOW);
      moveRect(0, (displacement_Y * 0.5), SPEED_SLOW);
      }
  }

  // Diagonal displacement
  else if (abs(displacement_X) == abs(displacement_Y)) moveDiag(displacement_X, displacement_Y, SPEED_SLOW);

  // Kingside castling
  else if (departure_coord_X == 5 && departure_coord_Y == 8 && arrival_coord_X == 7 && arrival_coord_Y == 8) {

    // Move the king off the board and behind the destination square
    moveRect(0.5, 0, SPEED_SLOW);
    moveRect(0, 2, SPEED_SLOW);
    electromagnet(false);

    // Move over to the rook, pick it up, and relocate
    moveRect(0, 1, SPEED_FAST);
    moveRect(-0.5, 0, SPEED_SLOW);
    electromagnet(true);
    moveRect(2, 0, SPEED_SLOW);
    electromagnet(false);

    // Now put the king in the final position
    moveDiag(1, 1, SPEED_SLOW);
    electromagnet(true);
    moveRect(0, -0.5, SPEED_SLOW);
  }

  // Queenside castling
  // TODO
  else if (departure_coord_X == 5 && departure_coord_Y == 8 && arrival_coord_X == 3 && arrival_coord_Y == 8) {
    // motor(R_L, SPEED_SLOW, 0.5);
    // motor(B_T, SPEED_SLOW, 2);
    // electromagnet(false);
    // motor(B_T, SPEED_FAST, 2);
    // motor(L_R, SPEED_FAST, 0.5);
    // electromagnet(true);
    // motor(T_B, SPEED_SLOW, 3);
    // electromagnet(false);
    // motor(B_T, SPEED_FAST, 1);
    // motor(R_L, SPEED_FAST, 0.5);
    // electromagnet(true);
    // motor(L_R, SPEED_SLOW, 0.5);
  }

  // Regular horizontal/vertical displacement
  else moveRect(displacement_X, displacement_Y, SPEED_SLOW);

  electromagnet(false);
}

// ===============================================================================
// MOVEMENT FUNCTIONS

void moveRectMM(long distanceX, long distanceY, int speed)
{
  long squaresX = distanceToSquares(distanceX);
  long squaresY = distanceToSquares(distanceY);
  moveRect(squaresX, squaresY, speed);
}

void moveRect(long squaresX, long squaresY, int speed)
{
  motorsSetSpeed(speed);

  long stepsX = squaresToSteps(squaresX);
  long stepsY = squaresToSteps(squaresY);
  // long steps[] = {-1*(stepsX + stepsY), (stepsX - stepsY)};
  // motPair.moveTo(steps);
  // motPair.runSpeedToPosition();

  // Complete the x-component
  if (stepsX > 0) {
    motA.moveTo(-stepsX);
    motB.moveTo(stepsX);
    motPair.runSpeedToPosition();
  }
  
  if (stepsY > 0) {
    // Complete the y-component
    motA.moveTo(stepsY);
    motB.moveTo(-stepsY);
    motPair.runSpeedToPosition();
  }
}

void moveDiag(long squaresX, long squaresY, int speed)
{
  motorsSetSpeed(speed);

  long stepsX = squaresToSteps(squaresX);
  long stepsY = squaresToSteps(squaresY);

  // Complete both x/y together
  motA.moveTo(-(stepsX + stepsY));
  motB.moveTo(stepsX - stepsY);
  motPair.runSpeedToPosition();
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
    for (byte j = 0; j < 4; j++) {
      // Set the select pins based on the table
      digitalWrite(MUX_SELECT[j], MUX_CHANNEL[i][j]);
      delay(1);
      // If we're looking at the first column of the bank...
      if (i < 8) {
        reed_sensor_status[i][j*2] = digitalRead(MUX_SIGNAL[j]);
      }
      // Otherwise, we must be looking at the second bank column...
      else {
        reed_sensor_status[i-8][(j*2)+1] = digitalRead(MUX_SIGNAL[j]);
      }
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

void motorsSetSpeed(int speed)
{
  if (speed == 1) {
    Serial.println("Set motor speed/accel to HIGH");
    motA.setMaxSpeed(VELOCITY_HIGH);
    motA.setAcceleration(ACCEL_HIGH);
    motB.setMaxSpeed(VELOCITY_HIGH);
    motB.setAcceleration(ACCEL_HIGH);
  }
  else {
    Serial.println("Set motor speed/accel to LOW");
    motA.setMaxSpeed(VELOCITY_LOW);
    motA.setAcceleration(ACCEL_LOW);
    motB.setMaxSpeed(VELOCITY_LOW);
    motB.setAcceleration(ACCEL_LOW);
  }
}

void zeroMotors()
{
  motA.setCurrentPosition(0);
  motB.setCurrentPosition(0);
  trolley_coordinate_X = 0;
  trolley_coordinate_Y = 0;
}

// ===============================================================================
// CONVERSION FUNCTIONS

long squaresToSteps(long squares)
{
  long distanceMM = squares * squareSize;
  long steps = distanceMM * stepsPerMM;
  return steps;
}

long distanceToSquares(int distanceMM)
{
  long squares = distanceMM / squareSize;
  return squares;
}
