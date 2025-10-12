
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

 

  // CAN initialization
  canW.begin();



  Serial.println("Setup complete. Waiting for CAN messages...");
}

void loop()
{
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


}



