
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "AbsoluteEncoder.h"
#include "Battery.h"
// #include "DynamixelSerial.h"
// #include "TractionEncoder.h"
// #include "MovingAvgFilter.h"
// #include "ExpSmoothingFilter.h"
#include "Debug.h"
#include "mcp2515.h"
#include "Display.h"

#include "CanWrapper.h"

#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"
#include <string.h> // strncpy, strlen

#include "Dynamixel_ll.h"

#include "debug_log.h"

#define SEND_DATA 100 // invio dati ogni 100 ms

CanWrapper canW(5, 20000000UL, &SPI);

float motor_L_command_data = 0.0f;
float motor_R_command_data = 0.0f;
float motor_LR_command_data[2] = {0.0f, 0.0f};

int time_sender = 0;
  int time_loop=0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB
    }

    SPI.setRX(4);
    SPI.setCS(5);
    SPI.setSCK(6);
    SPI.setTX(7);
    SPI.begin();

    Serial.println("CAN Exchange Example");

    // Additional setup code can be added here
}

void loop()
{
    uint8_t msg_id;
    uint8_t ID_MODULE;
    uint8_t CAN_OTHER_FLAG;
    byte msg_data[8];
     time_loop = millis();

    if (time_loop - time_sender  >= SEND_DATA)
    {
        time_sender = time_loop;
        // Example: send MOTOR_COMMAND message
       motor_LR_command_data[1]=motor_L_command_data;
       motor_LR_command_data[0]=motor_R_command_data;

        canW.sendMessage(MOTOR_SETPOINT, motor_LR_command_data, 8);
        /*
        #ifdef MODC_ARM
        arm_mot_1a1b_command_data
        canW.sendMessage(ARM_PITCH_1a1b_SETPOINT, arm_mot_1a1b_command_data, 8);
        canW.sendMessage(ARM_PITCH_2_SETPOINT, arm_mot_2_command_data, 4);
        canW.sendMessage(ARM_ROLL_3_SETPOINT, arm_mot_3_command_data, 4);
        canW.sendMessage(ARM_PITCH_4_SETPOINT, arm_mot_4_command_data, 4);
        canW.sendMessage(ARM_ROLL_5_SETPOINT, arm_mot_5_command_data, 4);
        canW.sendMessage(ARM_ROLL_6_SETPOINT, arm_mot_6_command_data, 4);
#endif*/
    }

    // helper lambdas per leggere valori little-endian
    auto read_u8 = [](const uint8_t *d, size_t off) -> uint8_t
    {
        return d[off];
    };
    auto read_u16_le = [](const uint8_t *d, size_t off) -> uint16_t
    {
        return (uint16_t)d[off] | ((uint16_t)d[off + 1] << 8);
    };
    auto read_u32_le = [](const uint8_t *d, size_t off) -> uint32_t
    {
        return (uint32_t)d[off] | ((uint32_t)d[off + 1] << 8) | ((uint32_t)d[off + 2] << 16) | ((uint32_t)d[off + 3] << 24);
    };

    if (canW.readMessage(&ID_MODULE, &CAN_OTHER_FLAG, &msg_id, msg_data))
    {
        switch (msg_id)
        {
        case MOTOR_FEEDBACK:
            Serial.print("MOTOR_FEEDBACK from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Speed Left: ");
            Serial.print(read_u16_le(msg_data, 0), DEC);
            Serial.print(" Speed Right: ");
            Serial.print(read_u16_le(msg_data, 4), DEC);
            Serial.println();

            break;
        case MOTOR_TRACTION_ERROR_STATUS:
            Serial.print("MOTOR_TRACTION_ERROR_STATUS from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Error status left: ");
            Serial.print(read_u8(msg_data, 0), DEC);
            Serial.print(" Error status right: ");
            Serial.print(read_u8(msg_data, 1), DEC);
            Serial.println();
            break;
        case ARM_PITCH_1a1b_FEEDBACK:
            Serial.print("ARM_PITCH_1a1b_FEEDBACK from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Position 1a: ");
            Serial.print(read_u16_le(msg_data, 0), DEC);
            Serial.print(" Position 1b: ");
            Serial.print(read_u16_le(msg_data, 4), DEC);
            Serial.println();
            break;
        case ARM_PITCH_2_FEEDBACK:
            Serial.print("ARM_PITCH_2_FEEDBACK from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Position 2: ");
            Serial.print(read_u16_le(msg_data, 0), DEC);
            Serial.println();
            break;
        case ARM_PITCH_4_FEEDBACK:
            Serial.print("ARM_PITCH_4_FEEDBACK from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Position 4: ");
            Serial.print(read_u16_le(msg_data, 0), DEC);
            Serial.println();
            break;

        case ARM_ROLL_5_FEEDBACK:
            Serial.print("ARM_ROLL_5_FEEDBACK from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Position 5: ");
            Serial.print(read_u16_le(msg_data, 0), DEC);
            Serial.println();
            break;
        case ARM_ROLL_6_FEEDBACK:
            Serial.print("ARM_ROLL_6_FEEDBACK from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Position 6: ");
            Serial.print(read_u16_le(msg_data, 0), DEC);
            Serial.println();
            break;

        case MOTOR_ARM_ERROR_STATUS:
            Serial.print("MOTOR_ARM_ERROR_STATUS from Module ID: ");
            Serial.print(ID_MODULE);
            Serial.print(" Other Flag: ");
            Serial.print(CAN_OTHER_FLAG);
            Serial.print(" Error status motor 1a: ");
            Serial.print(read_u8(msg_data, 0), DEC);
            Serial.print(" Error status motor 1b: ");
            Serial.print(read_u8(msg_data, 1), DEC);
            Serial.print(" Error status motor 2: ");
            Serial.print(read_u8(msg_data, 2), DEC);
            Serial.print(" Error status motor 3: ");
            Serial.print(read_u8(msg_data, 3), DEC);
            Serial.print(" Error status motor 4: ");
            Serial.print(read_u8(msg_data, 4), DEC);
            Serial.print(" Error status motor 5: ");
            Serial.print(read_u8(msg_data, 5), DEC);
            Serial.print(" Error status motor 6: ");
            Serial.print(read_u8(msg_data, 6), DEC);
            Serial.println();
            break;

        default:
            Serial.print("Received CAN message with ID: ");
            Serial.print(msg_id);
            Serial.print(" Data: ");
            for (int i = 0; i < 8; i++)
            {
                Serial.print(msg_data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            break;
        }
    }
}