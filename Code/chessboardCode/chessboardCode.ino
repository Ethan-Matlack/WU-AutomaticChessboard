// -----------------------------------------------------------------------------------------------
// --- AUTOMATED CHESSBOARD PROJECT --------------------------------------------------------------
/*

Widener University
RE-404: Mechatronics Lab
Final Project
2023-05-05

Team Members:
- Ethan Matlack
- Dimple Gandevia
- Chase Crane

TODO:
- Move MAGNET from 6 to A0
- Move mux signals to 6-9 and ground the common
- Verify that there's enough border-room to perform piece removal/castling

*/
// -----------------------------------------------------------------------------------------------
// --- LIBRARIES ---------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Micro_Max.h"
// #include <Mux.h>

// -----------------------------------------------------------------------------------------------
// --- CONSTANT DEFINITIONS ----------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
/* Define constants that will be used throughout the code. These values will not
change during program execution.
*/

// Pins
const byte LIMIT_SWITCH_X = 0;
const byte LIMIT_SWITCH_Y = 1;
const byte MUX_SELECT [4] = {2, 3, 4, 5};
const byte MUX_SIGNAL [4] = {6, 7, 8, 9};
// pins 10, 11, 12, 13 are used for motors and are defined using objects
const byte MAGNET = A0;

// Board
// All dimensions in mm
const float BOARD_SIZE = 457.2; // Size of the board's actual tiled area in mm
const float BOARD_BOUNDRY_TOP = 5; // Distance between top board edge and hard stop
const float BOARD_BOUNDRY_BOTTOM = 9.5; // Distance between bottom board edge and hard stop
const float BOARD_BOUNDRY_LEFT = 30; // Distance between left board edge and hard stop
const float BOARD_BOUNDRY_RIGHT = 16; // Distance between right board edge and hard stop
const float SQUARE_SIZE_MM = BOARD_SIZE / 8; // Size of each square in mm

// Motors
const word STEPS_PER_REV = 200;
const byte MICROSTEP_FACTOR = 4;
const float STEPS_PER_MM = 20.3; // Number of steps for each mm of motion (only applies to quarter-stepping rn...)
const byte SPEED_SLOW = 0;
const byte SPEED_FAST = 1;
const word VELOCITY_LOW = 450;
const word ACCEL_LOW = 150;
const word VELOCITY_HIGH = 900;
const word ACCEL_HIGH = 300;

// -----------------------------------------------------------------------------------------------
// --- VARIABLE DEFINITIONS ----------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

// Create the stepper objects
AccelStepper motA(1, 10, 11); // Motor A
AccelStepper motB(1, 12, 13); // Motor B
MultiStepper motPair; // Dual motor control

byte trolley_coordinate_X = 0; // Current X coordinate of trolley
byte trolley_coordinate_Y = 0; // Current Y coordinate of trolley

// Reed switch board status
bool reed_sensor_status [8][8]; // Current state of reed sensors
bool reed_sensor_status_memory [8][8]; // Previous state of reed sensors

// Multiplexer select truth table
const byte MUX_CHANNEL[16][4] = {

  // "Ideal" truth table, back half flipped
  // {0, 0, 0, 0},
  // {0, 0, 0, 1},
  // {0, 0, 1, 0},
  // {0, 0, 1, 1},
  // {0, 1, 0, 0},
  // {0, 1, 0, 1},
  // {0, 1, 1, 0},
  // {0, 1, 1, 1},
  // {1, 1, 1, 1},
  // {1, 1, 1, 0},
  // {1, 1, 0, 1},
  // {1, 1, 0, 0},
  // {1, 0, 1, 1},
  // {1, 0, 1, 0},
  // {1, 0, 0, 1},
  // {1, 0, 0, 0}

  // Actual? truth table, back half flipped
  {0, 0, 0, 0},
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {1, 1, 0, 0},
  {0, 0, 0, 1},
  {1, 0, 0, 1},
  {0, 1, 0, 1},
  {1, 1, 0, 1},
  {1, 1, 1, 1},
  {0, 1, 1, 1},
  {1, 0, 1, 1},
  {0, 0, 1, 1},
  {1, 1, 1, 0},
  {0, 1, 1, 0},
  {1, 0, 1, 0},
  {0, 0, 1, 0}
};

// Used for the main loop switch case
enum {calibration, player_white, player_black};
byte sequence = calibration;

// For tracking the board state changes (moves)
char mov [4] = {0, 0, 0, 0};
byte stateUpdate_col[2];
byte stateUpdate_row[2];

// MicroMax externals
extern char lastH[], lastM[];

// -----------------------------------------------------------------------------------------------
// --- FUNCTION PROTOYPES ------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
// These functions will be defined later in the code

void playComputerMove();
void moveLinMM(float distanceX, float distanceY, int speed);
void moveLin(float squaresX, float squaresY, int speed);
void moveDiag(float squaresX, float squaresY, int speed);
void electromagnet(bool state);
void readReedSwitches();
void motorsSetSpeed(int speed);
void zeroMotors();
float squaresToSteps(float squares);
float distanceToSquares(float distanceMM);

// -----------------------------------------------------------------------------------------------
// --- SETUP -------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

void setup()
{
  // Start the serial console
  Serial.begin(9600);
  
  // Add the steppers to a group
  motPair.addStepper(motA);
  motPair.addStepper(motB);

  // Digital pin modes
  pinMode(LIMIT_SWITCH_X, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y, INPUT_PULLUP);
  pinMode(MUX_SELECT[0], INPUT);
  pinMode(MUX_SELECT[1], INPUT);
  pinMode(MUX_SELECT[2], INPUT);
  pinMode(MUX_SELECT[3], INPUT);
  pinMode(MUX_SIGNAL[0], INPUT_PULLUP);
  pinMode(MUX_SIGNAL[1], INPUT_PULLUP);
  pinMode(MUX_SIGNAL[2], INPUT_PULLUP);
  pinMode(MUX_SIGNAL[3], INPUT_PULLUP);

  // Analog pin modes
  pinMode(MAGNET, OUTPUT);

  //  Set the reed switches status for a fresh game
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

// -----------------------------------------------------------------------------------------------
// --- MAIN LOOP ---------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

void loop()
{
  switch (sequence) {
    case calibration:
      serialBoard();
      calibrate();
      sequence = player_white;
      break;
    
    case player_white:
      Serial.println("Starting game...");
      if (checkForPlayerMove() == true) {
        decodePlayerMove();
        AI_HvsC();
        sequence = player_black;
        delay(500);
      }
      break;

    case player_black:
      playComputerMove();
      sequence = player_white;
      delay(500);
      readReedSwitches();
      break;
  }
}

// -----------------------------------------------------------------------------------------------
// --- CALIBRATION  ------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

void calibrate()
{
  // Run at a slow speed
  motorsSetSpeed(SPEED_SLOW);

  // Set a large goal for zero-ing the Y-axis and move towards it until hitting the limit switch
  motA.move(-20000);
  motB.move(20000);
  while (digitalRead(LIMIT_SWITCH_X) == HIGH) {
    motA.run();
    motB.run();
  }
  Serial.println("Limit reached for X-axis");
  delay(250);
  zeroMotors();
  
  // Set a large goal for zero-ing the Y-axis and move towards it until hitting the limit switch
  motA.move(20000);
  motB.move(20000);
  while (digitalRead(LIMIT_SWITCH_Y) == HIGH) {
    motA.run();
    motB.run();
  }
  Serial.println("Limited reached for Y-axis");
  delay(1000);
  zeroMotors();

  // Move to the lower left edge of the A1 square and zero
  moveDiagMM(-1*(BOARD_BOUNDRY_RIGHT + BOARD_SIZE), BOARD_BOUNDRY_BOTTOM, SPEED_FAST);
  zeroMotors();
  delay(3000);

  Serial.println("Calibration Complete!!");
}

// -----------------------------------------------------------------------------------------------
// --- PLAYER & COMPUTER MOVES -------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

bool checkForPlayerMove()
{
  bool flag = false;
  if ((digitalRead(LIMIT_SWITCH_X) == HIGH) || (digitalRead(LIMIT_SWITCH_Y) == HIGH)) {
    flag = true;
  }
  return flag;
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
  // mov[0] = table2[stateUpdate_row[0]];
  // mov[1] = table1[stateUpdate_col[0]];
  // mov[2] = table2[stateUpdate_row[1]];
  // mov[3] = table1[stateUpdate_col[1]];

  readMove(); // Call the readMove function
}

void playComputerMove()
/* This function is responsible for playing the computer's move on the chessboard. For simplicity,
we always assume that the computer is playing as black. The microMax notation is converted to
coordinates, capture is removed (if applicable), and the active piece is moved accordingly based
on its assumed type (derived from the different in reed switch board states.)
*/
{
  // Convert the algebraic notation of the move into coordinates.
  // For example, "e2e4" would be converted to (5,2) -> (5,4)
  int coord_depart_X = lastM[0] - 'a' + 1;
  int coord_depart_Y = lastM[1] - '0';
  int coord_arrival_X = lastM[2] - 'a' + 1;
  int coord_arrival_Y = lastM[3] - '0';

  // Initialize variables to record the change in coords for the moved piece
  int displacement_X = coord_arrival_X - coord_depart_X;
  int displacement_Y = coord_arrival_Y - coord_depart_Y;

  // If there was already a piece on the location that the new one is moving to, then
  // the move must be a capture. Get rid of the captured piece before resuming normally.
  if (reed_sensor_status_memory[coord_arrival_X][coord_arrival_Y] == 0) {
    Serial.println("Removing captured piece...");
    moveToDiag(coord_arrival_X - 0.5, coord_arrival_Y - 0.5, SPEED_FAST); // Move to the piece that gets removed
    electromagnet(true);
    moveLin(0.5, 0.5, SPEED_SLOW); // Shift diagonally to avoid hitting other pieces
    moveToLin(-0.2, 0, SPEED_SLOW); // Slide it off the board to the left side
    electromagnet(false);
  }

  // Move to the piece that will get moved
  moveToDiag(coord_depart_X - 0.5, coord_depart_Y - 0.5, SPEED_FAST);

  // Start the "main" piece movement code. Depending on the type of state change, different code
  // will be run to handle the different pieces and special cases. Start by turning the magnet on.

  electromagnet(true);

  // Knight displacement
  if ((abs(displacement_X) == 1 && abs(displacement_Y) == 2) || (abs(displacement_X) == 2 && abs(displacement_Y) == 1)) {
    if (abs(displacement_Y) == 2) {
      moveLin((displacement_X * 0.5), 0, SPEED_SLOW);
      moveLin(0, displacement_Y, SPEED_SLOW);
      moveLin((displacement_X * 0.5), 0, SPEED_SLOW);
    }
    else if (abs(displacement_X) == 2) {
      moveLin(0, (displacement_Y * 0.5), SPEED_SLOW);
      moveLin(displacement_X, 0, SPEED_SLOW);
      moveLin(0, (displacement_Y * -0.5), SPEED_SLOW);
      }
  }

  // Diagonal displacement
  else if (abs(displacement_X) == abs(displacement_Y)) {
    moveDiag(displacement_X, displacement_Y, SPEED_SLOW);
  }

  // Kingside castling
  else if (coord_depart_X == 5 && coord_depart_Y == 8 && coord_arrival_X == 7 && coord_arrival_Y == 8) {

    // Move the king off the board and behind the destination square
    moveLin(0.5, 0, SPEED_SLOW);
    moveLin(0, 2, SPEED_SLOW);
    electromagnet(false);

    // Move over to the rook, pick it up, and relocate
    moveDiag(-0.5, 1, SPEED_FAST);
    electromagnet(true);
    moveLin(2, 0, SPEED_SLOW);
    electromagnet(false);

    // Now put the king in the final position
    moveDiag(1, 1, SPEED_SLOW);
    electromagnet(true);
    moveLin(0, -0.5, SPEED_SLOW);
  }

  // Queenside (long) castling
  else if (coord_depart_X == 5 && coord_depart_Y == 8 && coord_arrival_X == 3 && coord_arrival_Y == 8) {
    
    // Move the king off the board and behind the destination square
    moveLin(0.5, 0, SPEED_SLOW);
    moveLin(0, -2, SPEED_SLOW);
    electromagnet(false);

    // Move over to the rook, pick it up, and relocate
    moveDiag(-0.5, -2, SPEED_FAST);
    electromagnet(true);
    moveLin(3, 0, SPEED_SLOW);
    electromagnet(false);

    // Now put the king in the final position
    moveDiag(-1, 1, SPEED_SLOW);
    electromagnet(true);
    moveLin(0, -0.5, SPEED_SLOW);
  }

  // Regular horizontal/vertical displacement
  else moveLin(displacement_X, displacement_Y, SPEED_SLOW);

  electromagnet(false);
}

// -----------------------------------------------------------------------------------------------
// --- SENSORS  ----------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

void readReedSwitches()
{
  Serial.println("Fetching new values...");

  // Read the values and store to an array
  for (byte i = 0; i < 16; i++) {
    for (byte j = 0; j < 4; j++) {
      // Set the select pins based on the table
      digitalWrite(MUX_SELECT[j], MUX_CHANNEL[i][j]);
      delay(5);
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
}

// -----------------------------------------------------------------------------------------------
// --- MOTOR HELPER FUNCTIONS --------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
/* These functions handle the movement of the trolley on the chessboard. moveLinMM is called when
the distances are given in millimeters, and it converts these values into squares before calling
moveLin. moveLin then converts the number of squares to the appropriate number of steps for each
motor, and moves the trolley accordingly using the motPair object. moveDiag is a helper function
used to move the trolley diagonally (e.g. for bishop moves), and moves the trolley by the
appropriate number of steps for both motors in order to achieve a diagonal motion.
*/

void moveLinMM(float distanceX, float distanceY, int speed)
{
  float squaresX = distanceToSquares(distanceX);
  float squaresY = distanceToSquares(distanceY);
  moveLin(squaresX, squaresY, speed);
}

void moveDiagMM(float distanceX, float distanceY, int speed)
{
  float squaresX = distanceToSquares(distanceX);
  float squaresY = distanceToSquares(distanceY);
  moveDiag(squaresX, squaresY, speed);
}

void moveLin(float squaresX, float squaresY, int speed)
{
  motorsSetSpeed(speed);

  float stepsX = squaresToSteps(squaresX);
  float stepsY = squaresToSteps(squaresY);

  if (abs(stepsX) > 0) {
    // Perform the x-translation
    motA.move(-stepsX);
    motB.move(stepsX);
    while ((abs(motA.distanceToGo()) > 0) || (abs(motB.distanceToGo()) > 0)) {
    motA.run();
    motB.run();
    }
  }
  
  if (abs(stepsY) > 0) {
    // Then do the y-translation
    motA.move(-stepsY);
    motB.move(-stepsY);
    while ((abs(motA.distanceToGo()) > 0) || (abs(motB.distanceToGo()) > 0)) {
    motA.run();
    motB.run();
    }
  }
}

void moveDiag(float squaresX, float squaresY, int speed)
{
  motorsSetSpeed(speed);

  float stepsX = squaresToSteps(squaresX);
  float stepsY = squaresToSteps(squaresY);
  float steps[] = {-1*(stepsX + stepsY), (stepsX - stepsY)};
  motA.move(steps[0]);
  motB.move(steps[1]);
  while ((abs(motA.distanceToGo()) > 0) || (abs(motB.distanceToGo()) > 0)) {
    motA.run();
    motB.run();
  }
}

void moveToLin(float squareCoordX, float squareCoordY, int speed)
{
  motorsSetSpeed(speed);

  float stepsX = squaresToSteps(squareCoordX);
  float stepsY = squaresToSteps(squareCoordY);

  if (abs(stepsX) > 0) {
    // Perform the x-translation
    motA.moveTo(-stepsX);
    motB.moveTo(stepsX);
    while ((abs(motA.distanceToGo()) > 0) || (abs(motB.distanceToGo()) > 0)) {
    motA.run();
    motB.run();
    }
  }
  
  if (abs(stepsY) > 0) {
    // Then do the y-translation
    motA.moveTo(-stepsY);
    motB.moveTo(-stepsY);
    while ((abs(motA.distanceToGo()) > 0) || (abs(motB.distanceToGo()) > 0)) {
    motA.run();
    motB.run();
    }
  }
}

void moveToDiag(float squareCoordX, float squareCoordY, int speed)
{
  motorsSetSpeed(speed);

  float stepsX = squaresToSteps(squareCoordX);
  float stepsY = squaresToSteps(squareCoordY);
  float steps[] = {-1*(stepsX + stepsY), (stepsX - stepsY)};
  motA.moveTo(steps[0]);
  motB.moveTo(steps[1]);
  while ((abs(motA.distanceToGo()) > 0) || (abs(motB.distanceToGo()) > 0)) {
    motA.run();
    motB.run();
  }
}

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

// -----------------------------------------------------------------------------------------------
// --- ELECTROMAGNET CONTROL ---------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
/* This function controls the electromagnet that is used to pick up and release the chess pieces.
When called with a "true" value, it powers the magnet and delays for 600 milliseconds to give it
time to activate. When called with a "false" value, it delays for 600 milliseconds and then turns
the magnet off.
*/

void electromagnet(bool state)
{
  if (state == true) {
    digitalWrite(MAGNET, HIGH);
    delay(600);
  }
  else {
    delay(600);
    digitalWrite(MAGNET, LOW);
  }
}

// -----------------------------------------------------------------------------------------------
// --- MISC CONVERTERS ---------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

float squaresToSteps(float squares)
{
  float distanceMM = squares * SQUARE_SIZE_MM;
  float steps = distanceMM * STEPS_PER_MM;
  return steps;
}

float distanceToSquares(float distanceMM)
{
  float squares = distanceMM / SQUARE_SIZE_MM;
  return squares;
}

void readMove() {
  bool validInput = false;

  while (!validInput) {
    Serial.println("Please enter a move:");

    while (Serial.available() == 0); // Wait for user input to become available

    String str = Serial.readStringUntil('\n');
    if (str.length() == 4) { // Check if the input has exactly four characters
      str.toCharArray(mov, str.length() + 1); // Correct the syntax for toCharArray()
      validInput = true; // Set validInput to true to exit the loop
      // Your code to process the input goes here
    } else {
      Serial.println("Invalid input. Please enter exactly four characters.");
    }

    delay(100); // Add a small delay to prevent constant looping
  }
}
