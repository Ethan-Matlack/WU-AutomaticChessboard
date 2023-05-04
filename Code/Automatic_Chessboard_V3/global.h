//  Chessboard
int reed_sensor_status [8][8];
int reed_sensor_record [8][8];
int reed_sensor_status_memory [8][8];
const float TROLLEY_START_POSITION_X = 0.78;
const float TROLLEY_START_POSITION_Y = 4.65;
byte trolley_coordinate_X = 5;
byte trolley_coordinate_Y = 7;
char mov [4] = {0, 0, 0, 0};
boolean no_valid_move = false;
byte reed_colone[2];
byte reed_line[2];

//  Game setting
enum {calibration, player_white, player_black};
byte sequence = calibration;
enum {T_B, B_T, L_R, R_L, LR_BT, RL_TB, LR_TB, RL_BT, calibrate_speed};
// T=Top  -  B=Bottom  -  L=Left  -  R=Right

//  Electromagnet
const byte MAGNET (6);

//  Countdown
byte second = 60;
byte minute = 9;
byte second_white = second;
byte second_black = second;
byte minute_white = minute;
byte minute_black = minute;
//unsigned long timer = 0;
boolean start_black = true;
boolean new_turn_countdown = false;

//  Motor
const byte MOTOR_WHITE_DIR (11);
const byte MOTOR_WHITE_STEP (10);
const byte MOTOR_BLACK_DIR (13);
const byte MOTOR_BLACK_STEP (12);
const byte SQUARE_SIZE = 195; //mm, change 
const int SPEED_SLOW (3000);
const int SPEED_FAST (1000);

//  Multiplexer
const byte MUX_ADDR [4] = {A3, A2, A1, A0}; //Select
const byte MUX_SELECT [4] = {5, 4, 3, 2}; //Signal - Output
//const byte MUX_OUTPUT (12); 
const byte MUX_CHANNEL[16][4] = {
  {0, 0, 0, 0},
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {1, 1, 0, 0},
  {0, 0, 1, 0},
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {1, 1, 1, 0},
  {1, 1, 1, 1}, //flipped
  {0, 1, 1, 1},
  {1, 0, 1, 1},
  {0, 0, 1, 1},
  {1, 1, 0, 1},
  {0, 1, 0, 1},
  {1, 0, 0, 1},
  {0, 0, 0, 1}
};

//  Button - Switch
const byte BUTTON_WHITE_SWITCH_MOTOR_WHITE (0);
const byte BUTTON_BLACK_SWITCH_MOTOR_BLACK (1);
enum {WHITE, BLACK};

//  MicroMax
extern char lastH[], lastM[];
