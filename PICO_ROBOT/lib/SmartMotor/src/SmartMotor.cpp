#line 1 "C:\\Users\\PC1\\Desktop\\ISAAC\\test_pid\\PicoLowLevel\\lib\\SmartMotor\\src\\SmartMotor.cpp"
#include "SmartMotor.h"
int pr = 0;
extern int motor_num;
/**
 * Create SmartMotor object, creating all necessary objects.
 * @param pwm PWM pin.
 * @param dir Direction pin.
 * @param enc_a Pin A of the encoder.
 * @param enc_b Pin B of the encoder.
 * @param invert Invert motor direction, usuful when motors are mounted opposite to one another.
 * @param pio PIO to use for the encoder. Each PIO can handle up to 4 encoders.
 */
SmartMotor::SmartMotor(byte pwm, byte dir, byte enc_a, byte enc_b, bool invert, PIO pio)
    : motor(pwm, dir, invert),
      encoder(enc_a, enc_b, new MovingAvgFilter<int>(ENC_TR_SAMPLES), invert, pio),
      pid(0.f, 0.f, 0.f, MAX_SPEED, 1.f),
      invert(invert)
{}

/**
 * Initialize SmartMotor and necessary components.
 */
void SmartMotor::begin() {
    motor.begin();
    encoder.begin();
}

/**
 * Update routine, updating the PID and the motor speed.
 * This function will be executed at a fixed rate, defined by DT_PID, and should therefore be called as often as possible.
 */
void SmartMotor::update() {
    unsigned long now = millis();
    if(now - pid_last > DT_PID) {
        if(motor_num==1) {
        Serial.print(" \tmillis\t");
  Serial.print(millis());

  // update motors
   Serial.print("\tMOTOR_LEFT\t");
    } else if(motor_num==2) {
        Serial.print(" \tmillis\t");
  Serial.print(millis());

  // update motors
   Serial.print("\tMOTOR_RIGHT\t");
}
        Serial.print("\tgetReferenceValue\t");
        Serial.print(pid.getReferenceValue());
        Serial.print("\tgetSpeed\t");
        Serial.print(getSpeed());
        pid.updateFeedback(getSpeed());
        pid.calculate();
        motor.write(speedToPower(pid.getOutput()));
        Serial.print("\tgetOutput\t");
        Serial.print(pid.getOutput());
        Serial.print("\tspeedtoPower\t");
       // Serial.print(pr);
       // pr++;
        Serial.println(speedToPower(pid.getOutput()));
        pid_last = now;
    }
}

/**
 * Set the desired speed of the motor.
 * @param value Desired motor speed between -MAX_SPEED and MAX_SPEED.
 */
void SmartMotor::setSpeed(float value) {
    pid.updateReferenceValue(value);
}

/**
 * Get the current speed of the motor.
 * The value is only updated at a fixed rate, defined by DT_ENC, to avoid losing precision.
 * @return float Current speed of the motor between -MAX_SPEED and MAX_SPEED.
 */
float SmartMotor::getSpeed() {
    unsigned long now = millis();
    if(now - enc_last > DT_ENC) {
        speed = (float)(encoder.getSpeed())/100.f;
        enc_last = now;
    }
    return speed;
}

/**
 * Stop the motor.
 * This function will stop the motor and reset the PID.
 */
void SmartMotor::stop() {
    motor.write(0);
    pid.updateReferenceValue(0.f);
    pid.resetState();
}

/**
 * Calibrate the PID controller.
 * This function will set the PID parameters to values that should work for the motor.
 * The method used is based on Åström–Hägglund tuning method, while using Ziegler-Nichols formulas to compute the gains.
 * Only the Kp and Ki gains are computed while the Kd gain is set to 0 since it doesn't have a positive effect on controlling the motor.
 * @param target Target speed to use for calibration.
 */
void SmartMotor::calibrate() {

    pid.setKp(0.35f);
    pid.setKi(1.0f);
    pid.setKd(0.15f);
}

/**
 * Converts speed to power.
 * This function linearly scales the speed to the PWM duty cycle, without taking into account the motor's.
 * @param speed Theretical speed of the motor.
 * @return int PWM value to set the motor to.
 */
int SmartMotor::speedToPower(float speed) {
    return (speed/MAX_SPEED)*PWM_MAX_VALUE;
}
