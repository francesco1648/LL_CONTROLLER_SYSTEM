
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "DynamixelSerial.h"
#include "TractionEncoder.h"
#include "MovingAvgFilter.h"
#include "ExpSmoothingFilter.h"
#include "Debug.h"
#include "mcp2515.h"
#include "Display.h"
#include "SmartMotor.h"
#include "Motor.h"
#include "PID.h"
#include "CanWrapper.h"

#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"

#include "Dynamixel_ll.h"

void okInterrupt();
void navInterrupt();
void sendFeedback();
void handleSetpoint(uint8_t msg_id, const byte *msg_data);
void DXL_TRACTION_INIT();
#ifdef MODC_ARM
void MODC_ARM_INIT();
void RESET_ARM_INITIAL_POSITION();
int32_t getClosestExtendedPosition(int32_t currentPos, int32_t targetPos);
#endif



int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;

CanWrapper canW(5, 20000000UL, &SPI);

//================ Traction Motors =================
DynamixelLL dxl_traction(Serial1, 0);
const uint8_t motorIDs_traction[] = {212, 114};
const uint8_t numMotors_traction = sizeof(motorIDs_traction) / sizeof(motorIDs_traction[0]);
DynamixelLL mot_Left_traction(Serial1, motorIDs_traction[0]);
DynamixelLL mot_Right_traction(Serial1, motorIDs_traction[1]);
float speeds_dxl[2] = {0.0f, 0.0f};
float old_speeds_dxl[2] = {0.0f, 0.0f};
float delta_speeds_dxl = 2.0f;
uint8_t data_dxl_traction[8];

int32_t currentSpeeds_left;  // Current speeds of the left traction motor for feedback
int32_t currentSpeeds_right; // Current speeds of the right traction motor for feedback

float currentSpeeds_left_float = 0.0f;
float currentSpeeds_right_float = 0.0f;

int32_t servo_data;

#ifdef MODC_YAW
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
#endif

#ifdef MODC_EE
DynamixelMotor motorEEPitch(SERVO_EE_PITCH_ID);
DynamixelMotor motorEEHeadPitch(SERVO_EE_HEAD_PITCH_ID);
DynamixelMotor motorEEHeadRoll(SERVO_EE_HEAD_ROLL_ID);
#endif
// Dichiarazione variabili per i motori del braccio
#ifdef MODC_ARM

const uint8_t motorIDs[] = {210, 211};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// variabili per la posizione iniziale
int32_t pos0_mot_2 = 0;
int32_t pos0_mot_3 = 0;
int32_t pos0_mot_4 = 0;
int32_t pos0_mot_5 = 0;
int32_t pos0_mot_6 = 0;
int32_t getpositions0[2] = {0, 0};

// variabili per la posizione attuale da mandare ai motori
int32_t pos_mot = 0;
int32_t pos_mot_2 = 0;
int32_t pos_mot_3 = 0;
int32_t pos_mot_4 = 0;
int32_t pos_mot_5 = 0;
int32_t pos_mot_6 = 0;
int32_t pos_mot_6_actual = 0;
int32_t getpositions[2] = {0, 0};
float theta_dxl;
float phi_dxl;
int32_t valueToSend = 0;

// variabili per i liminti di movimento
int32_t pos_mot_2_min = 0;
int32_t pos_mot_2_max = 0;
int32_t pos_mot_3_min = 0;
int32_t pos_mot_3_max = 0;
int32_t pos_mot_4_min = 0;
int32_t pos_mot_4_max = 0;
int32_t pos_mot_5_min = 0;
int32_t pos_mot_5_max = 0;
int32_t pos_mot_6_min = 0;
int32_t pos_mot_6_max = 0;

int32_t servo_data_mot_6 = 0; // variabile per la lettura della posizione del motore 6 dal CAN

// variabili per il feedback
int32_t posf_1a1b[2] = {0, 0};
int32_t posf_2 = 0;
int32_t posf_3 = 0;
int32_t posf_4 = 0;
int32_t posf_5 = 0;
int32_t posf_6 = 0;

float posf_1a1b_float[2] = {0.0f, 0.0f};
float posf_2_float = 0.0f;
float posf_3_float = 0.0f;
float posf_4_float = 0.0f;
float posf_5_float = 0.0f;
float posf_6_float = 0.0f;

// variabili per lettura dal CAN della posizione desiderata dei motori
float servo_data_1a = 0.0f;
float servo_data_1b = 0.0f;
float servo_data_float = 0.0f;

#define ProfileAcceleration 10
#define ProfileVelocity 20

int16_t presentLoad_mot_6 = 0;

DynamixelLL dxl(Serial1, 0);
DynamixelLL mot_Left_1(Serial1, motorIDs[0]);
DynamixelLL mot_Right_1(Serial1, motorIDs[1]);
DynamixelLL mot_2(Serial1, 112);
DynamixelLL mot_3(Serial1, 113);
DynamixelLL mot_4(Serial1, 214);
DynamixelLL mot_5(Serial1, 215);
DynamixelLL mot_6(Serial1, 216);

bool arm_roll_close_6_active = false;
bool arm_roll_open_6_active = false;

//========================================================
bool end_mot_6 = true; // reset end_mot_6 at each loop
int32_t target_pos_mot_6_open = -940;
int32_t target_pos_mot_6_close = -1600;
//========================================================

#endif

Display display;

void setup()
{

  Serial.begin(115200);

  Debug.setLevel(Levels::INFO); // comment to set debug verbosity to debug
  Debug.println("BEGIN", Levels::INFO);
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();

  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(6);
  SPI.setTX(7);
  SPI.begin();

  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(2000000); // Set baud rate for Dynamixel communication

  // CAN initialization
  canW.begin();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY);  // switching frequency to 15kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // initializing ADC
  analogReadResolution(12); // set precision to 12 bits, 0-4095 input
  display.begin();

  // Buttons initialization
  pinMode(BTNOK, INPUT_PULLUP);
  pinMode(BTNNAV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTNOK), okInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTNNAV), navInterrupt, FALLING);

  DXL_TRACTION_INIT();

#ifdef MODC_YAW
  encoderYaw.update();
  encoderYaw.readAngle();
  encoderYaw.setZero();
#endif

#ifdef MODC_ARM
  MODC_ARM_INIT();
#endif

  Serial.println("Setup complete. Waiting for CAN messages...");
}

void loop()
{
  int time_cur = millis();
  uint8_t msg_id;
  byte msg_data[8];

  // health checks
  if (time_cur - time_bat >= DT_BAT)
  {
    time_bat = time_cur;

    if (time_tel_avg > DT_TEL)
      // Debug.println("Telemetry frequency below required: " + String(1000 / time_tel_avg) + " Hz", Levels::WARN);

      if (!battery.charged())
        Debug.println("Battery voltage low! " + String(battery.readVoltage()) + "v", Levels::WARN);
  }

  // send telemetry
  if (time_cur - time_tel >= DT_TEL)
  {
    time_tel_avg = (time_tel_avg + (time_cur - time_tel)) / 2;
    time_tel = time_cur;

    sendFeedback();
    
   }

  if (canW.readMessage(&msg_id, msg_data))
  {

    // Received CAN message with setpoint
    time_data = time_cur;
    handleSetpoint(msg_id, msg_data);
  }
  else if (time_cur - time_data > CAN_TIMEOUT && time_data != -1)
  {
    // if we do not receive data for more than a second stop motors
    time_data = -1;

    // motorTrLeft.stop();
    speeds_dxl[0] = 0.0f;
    speeds_dxl[1] = 0.0f;

    dxl_traction.setGoalVelocity_RPM(speeds_dxl); // Stop both motors

    // motorTrRight.stop();
  }
  else
  {
  }

  // wm.handle();
  display.handleGUI();

#ifdef MODC_ARM
  if (arm_roll_close_6_active)
  {
    mot_6.getCurrentLoad(presentLoad_mot_6);
    Serial.print("presentLoad_mot_6 close");
    Serial.println(presentLoad_mot_6);

    mot_6.getPresentPosition(pos_mot_6_actual);
    Serial.print("pos_mot_6_actual: ");
    Serial.println(pos_mot_6_actual);

    Serial.print("target_pos_mot_6: ");
    Serial.println(target_pos_mot_6_close);


    if (end_mot_6)
    {

      mot_6.setGoalPosition_EPCM(target_pos_mot_6_close);
      end_mot_6 = 0;
    }
    else if (presentLoad_mot_6 >= 150 || abs(pos_mot_6_actual - target_pos_mot_6_close) <= 20)
    {

      arm_roll_close_6_active = false; // fine movimento
      arm_roll_open_6_active = false; // fine movimento
      mot_6.setGoalPosition_EPCM(pos_mot_6_actual);
    }
  }

  if (arm_roll_open_6_active)
  {
    mot_6.getCurrentLoad(presentLoad_mot_6);
    Serial.print("presentLoad_mot_6 open");
    Serial.println(presentLoad_mot_6);

    mot_6.getPresentPosition(pos_mot_6_actual);
    Serial.print("pos_mot_6_actual: ");
    Serial.println(pos_mot_6_actual);

    Serial.print("target_pos_mot_6: ");
    Serial.println(target_pos_mot_6_open);


    if (end_mot_6)
    {

      mot_6.setGoalPosition_EPCM(target_pos_mot_6_open);
      end_mot_6 = 0;
    }
    else if (presentLoad_mot_6 >= 150 || abs(pos_mot_6_actual - target_pos_mot_6_open) <=20)
    {

      arm_roll_open_6_active = false; // fine movimento
       arm_roll_close_6_active = false; // fine movimento
         mot_6.setGoalPosition_EPCM(pos_mot_6_actual);
    }
  }
#endif
}

/**
 * @brief Handles the setpoint messages received via CAN bus.
 * @param msg_id ID of the received message.
 * @param msg_data Pointer to the message data.
 */
void handleSetpoint(uint8_t msg_id, const byte *msg_data)
{

  switch (msg_id)
  {

  //========================================================
  case MOTOR_SETPOINT:
  {

    memcpy(&speeds_dxl[1], msg_data, 4);
    memcpy(&speeds_dxl[0], msg_data + 4, 4);

    // speeds_dxl[0] = speeds_dxl[0] * 0.667f; // adatto il massimo mandato dal telecomando (450.f) al massimo del motore (30 RPM)
    // speeds_dxl[1] = speeds_dxl[1] * 0.667f; // adatto il massimo mandato dal telecomando (450.f) al massimo del motore (30 RPM)
    /* if (abs(speeds_dxl - old_speeds_dxl) > delta_speeds_dxl)
     {
       dxl_traction.setGoalVelocity_RPM(speeds_dxl);
       old_speeds_dxl[0] = speeds_dxl[0];
       old_speeds_dxl[1] = speeds_dxl[1];
     }*/
    dxl_traction.setGoalVelocity_RPM(speeds_dxl);
    Debug.println("TRACTION DATA :\tleft: \t" + String(speeds_dxl[0]) + "\tright: \t" + String(speeds_dxl[1]));
    break;
  }

    //========================================================
  case DATA_EE_PITCH_SETPOINT:
    memcpy(&servo_data, msg_data, 4);
#ifdef MODC_EE
    motorEEPitch.moveSpeed(servo_data, SERVO_SPEED);
#endif
    Debug.print("PITCH END EFFECTOR MOTOR DATA : \t");
    Debug.println(servo_data);
    break;

    //========================================================
  case DATA_EE_HEAD_PITCH_SETPOINT:
    memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
    motorEEHeadPitch.moveSpeed(servo_data, SERVO_SPEED);
#endif
    Debug.print("HEAD PITCH END EFFECTOR MOTOR DATA : \t");
    Debug.println(servo_data);
    break;

    //========================================================
  case DATA_EE_HEAD_ROLL_SETPOINT:
    memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
    motorEEHeadRoll.moveSpeed(servo_data, SERVO_SPEED);
#endif
    Debug.print("HEAD ROLL END EFFECTOR MOTOR DATA : \t");
    Debug.println(servo_data);
    break;
#ifdef MODC_ARM
  //========================================================
  case ARM_PITCH_1a1b_SETPOINT:
    memcpy(&servo_data_1a, msg_data, 4);
    memcpy(&servo_data_1b, msg_data + 4, 4);
    /*Serial.print("servo_data_1a:\t");
    Serial.print(servo_data_1a);
    Serial.print("\tservo_data_1b:\t");
    Serial.println(servo_data_1b);*/
    theta_dxl = servo_data_1a;
    phi_dxl = servo_data_1b;
    getpositions[0] = (int32_t)(-((theta_dxl * (4096 / (2.0 * M_PI))) + (phi_dxl * (4096 / (2.0 * M_PI)))) / 2) + getpositions0[0];
    getpositions[1] = (int32_t)(((theta_dxl * (4096 / (2.0 * M_PI))) - (phi_dxl * (4096 / (2.0 * M_PI)))) / 2) + getpositions0[1];
/*
    Serial.print("getpositions[0]:\t");
    Serial.print(getpositions[0]);
    Serial.print("\tgetpositions[1]:\t");
    Serial.println(getpositions[1]);*/

    dxl.setGoalPosition_EPCM(getpositions);

    Debug.print("PITCH ARM 1a MOTOR DATA : \t");
    Debug.println(getpositions[0]);
    Debug.print("PITCH ARM 1b MOTOR DATA : \t");
    Debug.println(getpositions[1]);
    break;

    //========================================================
  case ARM_PITCH_2_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_2 = valueToSend + pos0_mot_2;
    /*
        // Check if the position is within the defined limits
        if(pos_mot < pos_mot_2_min){
          Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
          pos_mot_2 = pos_mot_2_min;
        }else if(pos_mot > pos_mot_2_max){
          Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
          pos_mot_2 = pos_mot_2_max;
        }else{
          pos_mot_2 = pos_mot;
        }*/

    mot_2.setGoalPosition_EPCM(pos_mot_2);

    Debug.print("PITCH ARM 2 MOTOR DATA : \t");
    Debug.println(pos_mot_2);
    break;

    //========================================================
  case ARM_ROLL_3_SETPOINT:


    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_3 = valueToSend + pos0_mot_3;
    /*
    // Check if the position is within the defined limits
    if(pos_mot < pos_mot_3_min){
      Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
      pos_mot_3 = pos_mot_3_min;
    }else if(pos_mot > pos_mot_3_max){
      Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
      pos_mot_3 = pos_mot_3_max;
    }else{
      pos_mot_3 = pos_mot;
    }

*/
    mot_3.setGoalPosition_EPCM(pos_mot_3);

    Debug.print("ROLL ARM 3 MOTOR DATA : \t");
    Debug.println(pos_mot_3);
    break;

    //========================================================
  case ARM_PITCH_4_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_4 = pos0_mot_4 + valueToSend;
    /*
            // Check if the position is within the defined limits
        if(pos_mot < pos_mot_4_min){
          Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
          pos_mot_4 = pos_mot_4_min;
        }else if(pos_mot > pos_mot_4_max){
          Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
          pos_mot_4 = pos_mot_4_max;
        }else{
          pos_mot_4 = pos_mot;
        }
    */
    mot_4.setGoalPosition_EPCM(pos_mot_4);

    Debug.print("PITCH ARM 4 MOTOR DATA : \t");
    Debug.println(pos_mot_4);
    break;

    //========================================================
  case ARM_ROLL_5_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_5 = pos0_mot_5 - valueToSend;
    /*
                // Check if the position is within the defined limits
        if(pos_mot < pos_mot_5_min){
          Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
          pos_mot_5 = pos_mot_5_min;
        }else if(pos_mot > pos_mot_5_max){
          Debug.println("ARM PITCH 2 SETPOINT OUT OF BOUNDS");
          pos_mot_5 = pos_mot_5_max;
        }else{
          pos_mot_5 = pos_mot;
        }
          */
    mot_5.setGoalPosition_EPCM(pos_mot_5);

    Debug.print("ROLL ARM 5 MOTOR DATA : \t");
    Debug.println(pos0_mot_5);
    break;

    //========================================================

    Debug.print("ROLL ARM 5 MOTOR DATA : \t");
    Debug.println(pos0_mot_5);
    break;
  case ARM_ROLL_6_SETPOINT:

    memcpy(&servo_data_mot_6, msg_data, 4);
    Serial.print("ARM ROLL 6 SETPOINT  ");
    Serial.println(servo_data_mot_6);
    if (servo_data_mot_6 == 0)
    {
      Serial.println("ARM ROLL 6 SETPOINT 1");

      arm_roll_close_6_active = true; // attiva la modalità di inseguimento
      arm_roll_open_6_active = false; // attiva la modalità di inseguimento
      end_mot_6 = true;               // reset end_mot_6 at each loop
    }
    if (servo_data_mot_6 == 1)
    {
      Serial.println("ARM ROLL 6 SETPOINT 0");
      arm_roll_close_6_active = false; // attiva la modalità di inseguimento
      arm_roll_open_6_active = true;   // attiva la modalità di inseguimento
      end_mot_6 = true;                // reset end_mot_6 at each loop
    }
    break;

  case RESET_ARM:
    RESET_ARM_INITIAL_POSITION();
    break;

#endif

#ifdef MODC_JOINT
    //========================================================
  case JOINT_PITCH_1d1s_SETPOINT:
    memcpy(&servo_data, msg_data, 2);

    dxlJOINT.setGoalPosition_EPCM(servo_data);

    Debug.print("PITCH JOINT 1d1s MOTOR DATA : \t");
    Debug.println(servo_data);
    break;
  case JOINT_ROLL_2_SETPOINT:
    memcpy(&servo_data, msg_data, 2);

    motorJOINT2Roll.setGoalPosition_EPCM(servo_data);

    Debug.print("ROLL JOINT 2 MOTOR DATA : \t");
    Debug.println(servo_data);
    break;
#endif
    //========================================================
  case MOTOR_TRACTION_REBOOT:
    mot_Left_traction.reboot();
    mot_Right_traction.reboot();
    Debug.println("Traction motors rebooted.");
    break;

  default:

    Debug.print("\tUnknown message ID:\t");

    Debug.print(msg_id);

    Debug.print("\tData:\t");
    for (int i = 0; i < 8; i++)
    {
      Debug.print(msg_data[i]);
      if (i < 7)
        Debug.print("\t");
    }
    Debug.print("\n");
    break;
  }
}

/**
 * @brief Sends feedback data over CAN bus.
 *
 * This function sends various feedback data including motor speeds, yaw angle, and end effector positions
 * if the respective modules are enabled.
 *
 * @note The function uses conditional compilation to include/exclude parts of the code based on the presence of specific modules.
 */
void sendFeedback()
{
  float speed_fb[2] = {currentSpeeds_left_float, currentSpeeds_right_float};
  dxl_traction.getPresentVelocity_RPM(speed_fb);

  memcpy(&data_dxl_traction[0], &speed_fb[0], 4); // copia il primo float nei primi 4 byte
  memcpy(&data_dxl_traction[4], &speed_fb[1], 4); // copia il secondo float nei secondi 4 byte

  canW.sendMessage(MOTOR_FEEDBACK, data_dxl_traction, 8);

  // send yaw angle of the joint if this module has one
#ifdef MODC_YAW
  encoderYaw.update();
  float angle = encoderYaw.readAngle();
  canW.sendMessage(JOINT_YAW_FEEDBACK, &angle, 4);
#endif

  // send end effector data (if module has it)
#ifdef MODC_EE
  int pitch = motorEEPitch.readPosition();
  int headPitch = motorEEHeadPitch.readPosition();
  int headRoll = motorEEHeadRoll.readPosition();

  canW.sendMessage(DATA_EE_PITCH_FEEDBACK, &pitch, 4);
  canW.sendMessage(DATA_EE_HEAD_PITCH_FEEDBACK, &headPitch, 4);
  canW.sendMessage(DATA_EE_HEAD_ROLL_FEEDBACK, &headRoll, 4);
#endif

  // Send the present position data of the arm motors
  /*
#ifdef MODC_ARM

  dxl.getPresentPosition(posf_1a1b);
  mot_2.getPresentPosition(posf_2);
  mot_3.getPresentPosition(posf_3);
  mot_4.getPresentPosition(posf_4);
  mot_5.getPresentPosition(posf_5);
  mot_6.getPresentPosition(posf_6);

  posf_1a1b_float[0] = (float)(posf_1a1b[0] * 1.0f);
  posf_1a1b_float[1] = (float)(posf_1a1b[1] * 1.0f);
  posf_2_float = (float)(posf_2 * 1.0f);
  posf_3_float = (float)(posf_3 * 1.0f);
  posf_4_float = (float)(posf_4 * 1.0f);
  posf_5_float = (float)(posf_5 * 1.0f);
  posf_6_float = (float)(posf_6 * 1.0f);

  canW.sendMessage(ARM_PITCH_1a1b_FEEDBACK, posf_1a1b_float, sizeof(posf_1a1b));
  canW.sendMessage(ARM_PITCH_2_FEEDBACK, &posf_2_float, sizeof(posf_2));
  canW.sendMessage(ARM_ROLL_3_FEEDBACK, &posf_3_float, sizeof(posf_3));
  canW.sendMessage(ARM_PITCH_4_FEEDBACK, &posf_4_float, sizeof(posf_4));
  canW.sendMessage(ARM_ROLL_5_FEEDBACK, &posf_5_float, sizeof(posf_5));
  canW.sendMessage(ARM_ROLL_6_FEEDBACK, &posf_6_float, sizeof(posf_6));


#endif

  // Send the present position data of the joint motors
#ifdef MODC_JOINT
  uint32_t pos_1d1s[numMotors] = {0, 0}; // Declare and initialize the array
  uint32_t pos_2 = 0;

  dxlJOINT.getPresentPosition(pos_1d1s);
  motorJOINT2Roll.getPresentPosition(pos_2);

  canW.sendMessage(JOINT_PITCH_1d1s_FEEDBACK, pos_1d1s, sizeof(pos_1d1s));
  canW.sendMessage(JOINT_ROLL_2_FEEDBACK, &pos_2, sizeof(pos_2));
#endif
*/
}

void okInterrupt()
{
  display.okInterrupt();
}

void navInterrupt()
{
  display.navInterrupt();
}
#ifdef MODC_ARM
void MODC_ARM_INIT()

{ // Initialize Dynamixel motors for the arm

  // Set the baud rate for Dynamixel communication
  dxl.begin_dxl(2000000);
  mot_Left_1.begin_dxl(2000000);
  mot_Right_1.begin_dxl(2000000);
  mot_2.begin_dxl(2000000);
  mot_3.begin_dxl(2000000);
  mot_4.begin_dxl(2000000);
  mot_5.begin_dxl(2000000);
  mot_6.begin_dxl(2000000);

  mot_Right_1.setTorqueEnable(false); // Disable torque for safety
  mot_Left_1.setTorqueEnable(false);
  mot_2.setTorqueEnable(false);
  mot_3.setTorqueEnable(false);
  mot_4.setTorqueEnable(false);
  mot_5.setTorqueEnable(false);
  mot_6.setTorqueEnable(false);

  delay(10);

  dxl.setStatusReturnLevel(2); // Set status return level for the main motor
  mot_Left_1.setStatusReturnLevel(2);
  mot_Right_1.setStatusReturnLevel(2);
  mot_2.setStatusReturnLevel(2);
  mot_3.setStatusReturnLevel(2);
  mot_4.setStatusReturnLevel(2);
  mot_5.setStatusReturnLevel(2);
  mot_6.setStatusReturnLevel(2);
  delay(10);

  // Enable or disable debug mode for troubleshooting
  mot_Left_1.setDebug(false);
  mot_Right_1.setDebug(false);
  mot_2.setDebug(false);
  mot_3.setDebug(false);
  mot_4.setDebug(false);
  mot_5.setDebug(false);
  mot_6.setDebug(false);
  dxl.setDebug(false);

  // Enable sync mode for multiple motor control.
  dxl.enableSync(motorIDs, numMotors);

  // Configure Drive Mode for each motor:
  mot_Left_1.setDriveMode(false, false, false);
  mot_Right_1.setDriveMode(false, false, false);
  mot_2.setDriveMode(false, false, false);
  mot_3.setDriveMode(false, false, false);
  mot_4.setDriveMode(false, false, false);
  mot_5.setDriveMode(false, false, false);
  mot_6.setDriveMode(false, false, false);

  // Set Operating Mode for each motor:
  dxl.setOperatingMode(4); // Extended Position Mode
  mot_2.setOperatingMode(4);
  mot_3.setOperatingMode(4);
  mot_4.setOperatingMode(4);
  mot_5.setOperatingMode(4);
  mot_6.setOperatingMode(4);

  /*
  mot1a 1780  mot1b 2957
  mot2 2122
  mot3 -1951
  mot4 1159
  mot5 5164
  mot6 -1098
  */

  delay(10);
  // Set Profile Velocity and Profile Acceleration for smooth motion.
  mot_Left_1.setProfileVelocity(ProfileVelocity);
  mot_Left_1.setProfileAcceleration(ProfileAcceleration);
  mot_Right_1.setProfileVelocity(ProfileVelocity);
  mot_Right_1.setProfileAcceleration(ProfileAcceleration);
  mot_2.setProfileVelocity(ProfileVelocity);
  mot_2.setProfileAcceleration(ProfileAcceleration);
  mot_3.setProfileVelocity(ProfileVelocity);
  mot_3.setProfileAcceleration(ProfileAcceleration);
  mot_4.setProfileVelocity(ProfileVelocity);
  mot_4.setProfileAcceleration(ProfileAcceleration);
  mot_5.setProfileVelocity(ProfileVelocity);
  mot_5.setProfileAcceleration(ProfileAcceleration);
  mot_6.setProfileVelocity(ProfileVelocity);
  mot_6.setProfileAcceleration(ProfileAcceleration);

  delay(10);
  // Enable torque for all motors.
  dxl.setTorqueEnable(true);
  mot_Left_1.setTorqueEnable(true);
  mot_Right_1.setTorqueEnable(true);
  mot_2.setTorqueEnable(true);
  mot_3.setTorqueEnable(true);
  mot_3.setLED(true); // Enable LED for visual feedback
  mot_4.setTorqueEnable(true);
  mot_5.setTorqueEnable(true);
  mot_6.setTorqueEnable(true);
delay(10);
getpositions0[0] = 2209;
getpositions0[1] = 1451;
pos0_mot_2 = 4746;
pos0_mot_3 = 987;
pos0_mot_4 = 3121;
pos0_mot_5 = 1979;
pos0_mot_6 = 0;
  RESET_ARM_INITIAL_POSITION();
}

void RESET_ARM_INITIAL_POSITION()
{
  // Leggi posizioni correnti
  int32_t posCurr0[2] = {0, 0};
  int32_t posCurr2 = 0;
  int32_t posCurr3 = 0;
  int32_t posCurr4 = 0;
  int32_t posCurr5 = 0;
  int32_t posCurr6 = 0;

  dxl.getPresentPosition(posCurr0);
  mot_2.getPresentPosition(posCurr2);
  mot_3.getPresentPosition(posCurr3);
  mot_4.getPresentPosition(posCurr4);
  mot_5.getPresentPosition(posCurr5);
  mot_6.getPresentPosition(posCurr6);

  // Calcola posizione più vicina con funzione helper
  int32_t posTarget0_0 = getClosestExtendedPosition(posCurr0[0], getpositions0[0]);
  int32_t posTarget0_1 = getClosestExtendedPosition(posCurr0[1], getpositions0[1]);
  int32_t posTarget2 = getClosestExtendedPosition(posCurr2, pos0_mot_2);
  int32_t posTarget3 = getClosestExtendedPosition(posCurr3, pos0_mot_3);
  int32_t posTarget4 = getClosestExtendedPosition(posCurr4, pos0_mot_4);
  int32_t posTarget5 = getClosestExtendedPosition(posCurr5, pos0_mot_5);
  int32_t posTarget6 = getClosestExtendedPosition(posCurr6, pos0_mot_6);

  // Ora assegna le posizioni “aggiustate” (assumendo che dxl gestisca 2 motori per esempio)
  int32_t posTargets0[2] = {posTarget0_0, posTarget0_1};
  dxl.setGoalPosition_EPCM(posTargets0);
  mot_2.setGoalPosition_EPCM(posTarget2);
  mot_3.setGoalPosition_EPCM(posTarget3);
  mot_4.setGoalPosition_EPCM(posTarget4);
  mot_5.setGoalPosition_EPCM(posTarget5);
  mot_6.setGoalPosition_EPCM(posTarget6);
}

int32_t getClosestExtendedPosition(int32_t currentPos, int32_t targetPos)
{
  const int32_t oneRevolution = 4096; // numero di unità per un giro completo
  int32_t diff = targetPos - currentPos;

  // Modulo della differenza per tenerla nell’intervallo [-oneRevolution/2, oneRevolution/2]
  diff = ((diff + oneRevolution / 2) % oneRevolution) - oneRevolution / 2;

  return currentPos + diff;
}

#endif

void DXL_TRACTION_INIT()
{
  // Initialize Dynamixel motors for the arm

  // Set the baud rate for Dynamixel communication
  dxl_traction.begin_dxl(2000000);
  mot_Left_traction.begin_dxl(2000000);
  mot_Right_traction.begin_dxl(2000000);

  mot_Right_traction.setTorqueEnable(false); // Disable torque for safety
  mot_Left_traction.setTorqueEnable(false);

  delay(10);

  dxl_traction.setStatusReturnLevel(2); // Set status return level for the main motor
  mot_Left_traction.setStatusReturnLevel(2);
  mot_Right_traction.setStatusReturnLevel(2);

  delay(10);

  // Enable or disable debug mode for troubleshooting
  mot_Left_traction.setDebug(false);
  mot_Right_traction.setDebug(false);
  dxl_traction.setDebug(false);

  // Enable sync mode for multiple motor control.
  dxl_traction.enableSync(motorIDs_traction, numMotors_traction);

  // Configure Drive Mode for each motor:
  mot_Left_traction.setDriveMode(false, false, true);
  mot_Right_traction.setDriveMode(false, false, false);

  // Set Operating Mode for each motor:
  dxl_traction.setOperatingMode(1); // Extended Position Mode

  delay(10);
  // Set Profile Velocity and Profile Acceleration for smooth motion.

  // Enable torque for all motors.
  dxl_traction.setTorqueEnable(true);
  mot_Left_traction.setTorqueEnable(true);
  mot_Right_traction.setTorqueEnable(true);
}