#ifndef MOTORS_H
#define MOTORS_H

#include <avr/io.h>

extern const double original_speed;
extern const double distance_threshold; // in inches (was 20.0 cm, ~7.87 in)
extern double       percent_speed;


/* Direction constants */
#define FORWARD   1
#define BACKWARD -1

/* Motor identifiers */
#define MOTOR_LEFT   0
#define MOTOR_RIGHT  1

/* This value scales floating “speed” into 0–255 or 0–MOTOR_TOP_SPEED range.
   If using an 8-bit timer with Fast PWM (0–255 range), adjust accordingly. */
#define MOTOR_TOP_SPEED 200

/* Motor control pins (adjust as needed by hardware) */
#define LEFT_CONTROL PD2   // direction for left motor
#define LEFT_PWM     PD5   // PWM pin for left motor
#define RIGHT_CONTROL PD4  // direction for right motor
#define RIGHT_PWM     PD6  // PWM pin for right motor

void enable_motors(void);
void setup_motors(void);
void set_motor_speed(int motor, double speed, int direction);
void follow_wall(double power_percentage, double maintain_distance, int side);

#endif
