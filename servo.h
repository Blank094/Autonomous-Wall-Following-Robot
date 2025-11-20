#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_PIN PB2

#define SERVO_RIGHT   150   // ~ -90 deg
#define SERVO_LEFT    625   // ~ +90 deg
#define SERVO_FRONT  375   //   0 deg

void setup_servo(void);
void move_servo(int pulseWidth);

#endif
