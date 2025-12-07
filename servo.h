#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_PIN PB2

#define SERVO_RIGHT   150   // ~ -90 deg (looking right)
#define SERVO_LEFT    625   // ~ +90 deg (looking left)
#define SERVO_FRONT   350   //   0 deg (looking forward)

void setup_servo(void);

// Move servo - returns 1 if move was needed, 0 if already at position
uint8_t move_servo(int pulseWidth);

// Smart move with calculated delay - returns suggested wait time via pointer
uint8_t move_servo_smart(int pulseWidth, uint8_t *wait_ms);

// Check if servo is already at target position
uint8_t servo_at_position(int pulseWidth);

// Get current servo position
int get_servo_position(void);

#endif
