#include "Motor.h"

/**
 * Create object and set motor pins.
 * @param pwm PWM pin.
 * @param dir Direction pin.
 * @param invert Invert motor direction, usuful when motors are mounted opposite to one another.
 */
Motor::Motor(byte pwm, byte dir, bool invert)
    : pwm(pwm), dir(dir), invert(invert)
{}

/**
 * Initialize motors.
 */
void Motor::begin() {
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

/**
 * Sets the motor speed.
 * @param value Speed of the motor, ranging from 0 to maximum PWM value.
 */
void Motor::write(int value) {
  static float filtered_value = 0;  // persiste tra le chiamate

  // Parametro di filtro: 0.0 = filtro forte, 1.0 = nessun filtro
  const float alpha = 0.2;

  // Applica filtro sull'ingresso
  filtered_value = alpha * value + (1.0 - alpha) * filtered_value;




  // Estrai direzione e modulo
  int sign = (filtered_value < 0) ? -1 : (filtered_value > 0 ? 1 : 0);
  int mot = constrain(abs((int)filtered_value), 0, PWM_MAX_VALUE);

  const int MIN_PWM = 50;

  if (mot > 0 && mot < MIN_PWM) {
    mot = 0;
  }

  analogWrite(pwm, mot);
  digitalWrite(dir, invert ^ (sign < 0));

  Serial.print("\tanalogWrite\t");
  Serial.print(mot);
  Serial.print("\tdigitalWrite\t");
  Serial.print(invert ^ (sign < 0));
}


