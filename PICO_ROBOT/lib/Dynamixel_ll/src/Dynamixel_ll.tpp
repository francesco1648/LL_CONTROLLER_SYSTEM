#pragma once
#define time_delay 0

template <typename T>
uint8_t DynamixelLL::readRegister(uint16_t address, T &value, uint8_t size)
{
    // Build a 14-byte READ instruction packet:
    // [Header (4) | Servo ID (1) | Length (2) | Instruction (1) | Parameters (4) | CRC (2)]
    uint8_t packet[14];
    uint16_t length = 7;  // Parameter bytes (4) + Instruction (1) + CRC (2)

    // Header (4 bytes):
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // Servo ID (1 byte):
    packet[4] = _servoID;

    // Length field (2 bytes, little-endian):
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;

    // Instruction (1 byte): READ (0x02)
    packet[7] = 0x02;

    // Parameters (4 bytes): starting address and data length (each in little-endian)
    packet[8] = address & 0xFF;
    packet[9] = (address >> 8) & 0xFF;
    packet[10] = size & 0xFF;
    packet[11] = (size >> 8) & 0xFF;

    // Compute and append CRC (over the first 12 bytes)
    uint16_t crc = calculateCRC(packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    // Transmit the packet.
    if (!sendPacket(packet, 14))
    {
        if (_debug)
            Serial.println("Error sending Read packet.");
        return 1;
    }
    delay(time_delay);

    // Receive and process the response.
    StatusPacket response = receivePacket();
    if (_debug)
    {
        if (!response.valid)
            Serial.println("Invalid status packet received.");
        if (response.error != 0)
        {
            Serial.print("Error in status packet: ");
            Serial.println(response.error, HEX);
        }
    }

    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++)
        value |= (response.data[i] << (8 * i));

    delay(time_delay);
    return response.error;
}


template <typename T>
uint8_t DynamixelLL::syncRead(uint16_t address, uint8_t dataLength, const uint8_t* ids, T* values, uint8_t count)
{
    // Send Sync Read Instruction Packet.
    if (!sendSyncReadPacket(address, dataLength, ids, count))
    {
        if (_debug)
            Serial.println("Error sending Sync Read packet.");
        return 1;
    }

    uint8_t retError = 0;
    for (uint8_t i = 0; i < count; i++)
        values[i] = 0;
    // For each device, read its response.
    uint8_t received = 0;
    while (received < count)
    {
        StatusPacket response = receivePacket();
        received++;
        if (!response.valid)
        {
            if (_debug)
                Serial.println("Invalid status packet received.");
            continue;
        }
        if (response.error != 0)
        {
            if (_debug)
            {
                Serial.print("Error in status packet from device ");
                Serial.print(response.id);
                Serial.print(": 0x");
                Serial.println(response.error, HEX);
            }
            retError = response.error;
            continue;
        }
        // Find the index in the provided ids array that matches the response id.
        for (uint8_t i = 0; i < count; i++)
        {
            if (ids[i] == response.id)
            {
                for (uint8_t j = 0; j < response.dataLength; j++)
                    values[i] |= (response.data[j] << (8 * j));
                break;
            }
        }
    }
    return retError;
}


template <uint8_t N>
uint8_t DynamixelLL::setOperatingMode(const uint8_t (&modes)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedModes[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    { 
        if (!(modes[i] == 1 || modes[i] == 3 || modes[i] == 4 || modes[i] == 16))
        {
            if (_debug)
                Serial.print("Error: Unsupported operating mode.");
            return 1;
        }
        processedModes[i] = modes[i];
    }
    return syncWrite(11, 1, _motorIDs, processedModes, _numMotors); // EEPROM address 11, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setHomingOffset(const int32_t (&offset)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t offsetArray[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        if (offset[i] > 1044479)
        {
            offsetArray[i] = 1044479;
            if (_debug)
                Serial.println("Warning: Homing offset clamped to 1044479.");
        } else if (offset[i] < -1044479) {
            offsetArray[i] = -1044479;
            if (_debug)
                Serial.println("Warning: Homing offset clamped to -1044479.");
        } else
            offsetArray[i] = offset[i];
    }
    return syncWrite(20, 4, _motorIDs, offsetArray, _numMotors); // RAM address 20, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setHomingOffset_A(const float (&offsetAngle)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    int32_t offsetPulse[_numMotors];
    uint32_t offsetArray[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
        offsetPulse[i] = static_cast<int32_t>(offsetAngle[i] / 0.088);
        if (offsetPulse[i] > 1044479)
        {
            offsetArray[i] = 1044479;
            if (_debug)
                Serial.println("Warning: Homing offset clamped to 1044479.");
        } else if (offsetPulse[i] < -1044479) {
            offsetArray[i] = -1044479;
            if (_debug)
                Serial.println("Warning: Homing offset clamped to -1044479.");
        } else
            offsetArray[i] = static_cast<uint32_t>(offsetPulse[i]);
    }
    return syncWrite(20, 4, _motorIDs, offsetArray, _numMotors); // RAM address 20, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setGoalPosition_PCM(const uint16_t (&goalPositions)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedPositions[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        processedPositions[i] = goalPositions[i];

        if (processedPositions[i] > 4095) {
            processedPositions[i] = 4095;
            if (_debug)
                Serial.println("Warning: Goal position clamped to 4095.");
        }
    }
    return syncWrite(116, 4, _motorIDs, processedPositions, _numMotors); // RAM address 116, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setGoalPosition_A_PCM(const float (&angleDegrees)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedPositions[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
        processedPositions[i] = static_cast<uint32_t>(angleDegrees[i] / 0.088);
        if (processedPositions[i] > 4095) {
            processedPositions[i] = 4095;
            if (_debug)
                Serial.println("Warning: Angle conversion resulted in value exceeding 4095, clamped.");
        }
    }
    return syncWrite(116, 4, _motorIDs, processedPositions, _numMotors); // RAM address 116, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setGoalPosition_EPCM(const int32_t (&extendedPositions)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedPositions[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        if (extendedPositions[i] > 1048575)
        {
            processedPositions[i] = 1048575;
            if (_debug)
                Serial.println("Warning: Extended position clamped to 1048575.");
        } else if (extendedPositions[i] < -1048575) {
            processedPositions[i] = -1048575;
            if (_debug)
                Serial.println("Warning: Extended position clamped to -1048575.");
        } else
            processedPositions[i] = static_cast<uint32_t>(extendedPositions[i]);
    }
    return syncWrite(116, 4, _motorIDs, processedPositions, _numMotors); // RAM address 116, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setTorqueEnable(const bool (&enable)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedValues[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
        processedValues[i] = enable[i] ? 1 : 0;
    return syncWrite(64, 1, _motorIDs, processedValues, _numMotors); // RAM address 64, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setLED(const bool (&enable)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedValues[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
        processedValues[i] = enable[i] ? 1 : 0;
    return syncWrite(65, 1, _motorIDs, processedValues, _numMotors); // RAM address 65, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setStatusReturnLevel(const uint8_t (&levels)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedLevels[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        if (levels[i] > 2)
        {
            if (_debug)
                Serial.println("Error: Invalid status return level. Allowed values: 0, 1, or 2.");
            return 1;
        }
        processedLevels[i] = levels[i];
    }
    return syncWrite(68, 1, _motorIDs, processedLevels, _numMotors); // RAM address 68, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setID(const uint8_t (&newIDs)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedIDs[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        if (newIDs[i] > 253)
        {
            if (_debug)
                Serial.println("Error: Invalid ID. Valid IDs are 0 to 253.");
            return 1;
        }
        processedIDs[i] = newIDs[i];
    }
    return syncWrite(7, 1, _motorIDs, newIDs, _numMotors); // EEPROM address 7, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setBaudRate(const uint8_t (&baudRates)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedBaudRates[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        const uint8_t allowed[] = {0, 1, 2, 3, 4, 5, 6, 7};
        bool valid = false;
        
        for (uint8_t j = 0; j < sizeof(allowed) / sizeof(allowed[0]); j++)
        {
            if (allowed[j] == baudRates[i])
            {
                valid = true;
                break;
            }
        }
        
        if (!valid)
        {
            if (_debug)
            {
                Serial.print("Error: Unrecognized baud rate code: ");
                Serial.println(baudRates[i]);
            }
            return 1;
        }
        processedBaudRates[i] = baudRates[i];
    }
    return syncWrite(8, 1, _motorIDs, processedBaudRates, _numMotors); // EEPROM address 8, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setReturnDelayTime(const uint8_t (&delayTime)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedDelayTime[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        processedDelayTime[i] = delayTime[i];
        if (processedDelayTime[i] > 254)
        {
            processedDelayTime[i] = 254;
            if (_debug)
                Serial.println("Warning: setReturnDelayTime clamped to 254.");
        }
    }
    return syncWrite(9, 1, _motorIDs, processedDelayTime, _numMotors); // EEPROM address 9, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setDriveMode(const bool (&torqueOnByGoalUpdate)[N],
                                  const bool (&timeBasedProfile)[N],
                                  const bool (&reverseMode)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedDriveModes[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        uint8_t mode = 0;
        if (torqueOnByGoalUpdate[i])
            mode |= 0x08; // Set Bit 3.
        if (timeBasedProfile[i])
            mode |= 0x04; // Set Bit 2.
        if (reverseMode[i])
            mode |= 0x01; // Set Bit 0.
        processedDriveModes[i] = mode;
    }
    return syncWrite(10, 1, _motorIDs, processedDriveModes, _numMotors); // EEPROM address 10, 1 byte
}


template <uint8_t N>
uint8_t DynamixelLL::setProfileVelocity(const uint32_t (&profileVelocity)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedProfileVelocity[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        uint8_t driveMode = 0;
        uint8_t error = readRegister(10, driveMode, 1);
        // Bit 2 (0x04) set indicates time-based profile.
        bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);
    
        // Select maximum allowed velocity based on profile type.
        const uint32_t maxProfileVelocity = timeBased ? 32737UL : 32767UL;
        
        if (profileVelocity > maxProfileVelocity)
        {
            if (_debug)
            {
                Serial.print("Profile velocity clamped to ");
                Serial.println(maxProfileVelocity);
            }
            processedProfileVelocity[i] = maxProfileVelocity;
        } else
            processedProfileVelocity[i] = profileVelocity[i];
    }
    return syncWrite(112, 4, _motorIDs, processedProfileVelocity, _numMotors); // RAM address 112, 4 bytes
}


template <uint8_t N>
uint8_t DynamixelLL::setProfileAcceleration(const uint32_t (&profileAcceleration)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    uint32_t processedProfileAcceleration[_numMotors];
    for (uint8_t i = 0; i < _numMotors; i++) // Iterate through all motors
    {
        // Read drive mode from register 10 to determine if a time-based profile is active.
        uint8_t driveMode = 0;
        uint8_t error = readRegister(10, driveMode, 1);
        bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);
        
        // Choose the maximum allowed acceleration based on profile type.
        const uint32_t maxProfileAcceleration = timeBased ? 32737UL : 32767UL;
        
        if (profileAcceleration > maxProfileAcceleration)
        {
            if (_debug)
            {
                Serial.print("Profile acceleration clamped to ");
                Serial.println(maxProfileAcceleration);
            }
            processedProfileAcceleration[i] = maxProfileAcceleration;
        } else
            processedProfileAcceleration[i] = profileAcceleration[i];
        
        // For time-based profiles, ensure that acceleration does not exceed half of the current profile velocity.
        uint32_t currentProfileVelocity = 0;
        error = readRegister(112, currentProfileVelocity, 4);
        if (timeBased && error == 0 && currentProfileVelocity > 0 && profileAcceleration > (currentProfileVelocity / 2))
        {
            uint32_t clampedValue = currentProfileVelocity / 2;
            if (_debug)
            {
                Serial.print("Profile acceleration clamped to half of current profile velocity: ");
                Serial.println(clampedValue);
            }
            processedProfileAcceleration[i] = clampedValue;
        } else if (error != 0 && _debug) {
            Serial.print("Error reading Profile Velocity: ");
            Serial.println(error);
        } else
            processedProfileAcceleration[i] = profileAcceleration[i];
    }
    return syncWrite(118, 4, _motorIDs, processedProfileAcceleration, _numMotors); // RAM address 108, 4 bytes
}

template <uint8_t N>
uint8_t DynamixelLL::setGoalVelocity_RPM(const float (&rpmValues)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    const float maxRPM = 30.0f; // stimato per 12V
    const float rpmToUnit = 1.0f / 0.229f; // ≈ 4.3668 unità per RPM

    uint32_t processedValues[_numMotors];

    for (uint8_t i = 0; i < _numMotors; i++)
    {
        float rpm = rpmValues[i];

        // Clamping dell'RPM
        if (rpm > maxRPM)
        {
            rpm = maxRPM;
            if (_debug)
            {
                Serial.print("Warning: RPM clamped to ");
                Serial.println(maxRPM);
            }
        }
        else if (rpm < -maxRPM)
        {
            rpm = -maxRPM;
            if (_debug)
            {
                Serial.print("Warning: RPM clamped to ");
                Serial.println(-maxRPM);
            }
        }

        int16_t velocityUnits = static_cast<int16_t>(rpm * rpmToUnit);
        processedValues[i] = static_cast<uint32_t>(velocityUnits);
    }

    return syncWrite(104, 4, _motorIDs, processedValues, _numMotors); // RAM address 104, 4 bytes
}

template <uint8_t N>
uint8_t DynamixelLL::getPresentPosition(int32_t (&presentPositions)[N])
{
    if (checkArraySize(N) != 0)
        return 1;
    
    uint8_t error = syncRead(132, 4, _motorIDs, presentPositions, _numMotors); // RAM address 132, 4 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Present Position: ");
            Serial.println(error);
        }
    }
    return error;
}


template <uint8_t N>
uint8_t DynamixelLL::getCurrentLoad(int16_t (&currentLoad)[N])
{
    if (checkArraySize(N) != 0)
        return 1;
    
    uint8_t error = syncRead(126, 2, _motorIDs, currentLoad, _numMotors); // RAM address 126, 2 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Current Load: ");
            Serial.println(error);
        }
    }
    return error;
}

template <uint8_t N>
uint8_t DynamixelLL::getMovingStatus(MovingStatus (&status)[N])
{
    if (checkArraySize(N) != 0)
        return 1;
    
    uint8_t temp[_numMotors];
    uint8_t error = syncRead(123, 1, _motorIDs, temp, _numMotors); // RAM address 123, 1 byte
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Moving Status: ");
            Serial.println(error);
        }
    } else
    {
        for (uint8_t i = 0; i < _numMotors; i++)
        {
            // Extract the status byte.
            status[i].raw = temp[i];

            // Decode bits 5 & 4 for Velocity Profile Type.
            uint8_t profileBits = (status[i].raw >> 4) & 0x03;
            status[i].profileType = static_cast<VelocityProfileType>(profileBits);

            // Decode bit 3 for Following Error.
            status[i].followingError = ((status[i].raw >> 3) & 0x01) != 0;
            // Decode bit 1 to check if a motion profile is ongoing.
            status[i].profileOngoing = ((status[i].raw >> 1) & 0x01) != 0;
            // Decode bit 0 to determine if target position is reached.
            status[i].inPosition = (status[i].raw & 0x01) != 0;
        }
    }
    return error;
}

template <uint8_t N>
uint8_t DynamixelLL::getPresentVelocity_RPM(float (&rpms)[N])
{
    if (checkArraySize(N) != 0)
        return 1;

    int16_t temp[_numMotors];
    uint8_t error = syncRead(128, 4, _motorIDs, temp, _numMotors); // RAM address 128, 4 bytes
    if (error != 0)
    {
        if (_debug)
        {
            Serial.print("Error reading Present Velocity: ");
            Serial.println(error);
        }
    } else {
        for (uint8_t i = 0; i < _numMotors; i++)
            rpms[i] = static_cast<float>(temp[i]) * 0.229f;  // convert to RPM in float
    }
    return error;
}