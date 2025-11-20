#include "motors.h"
#include "ultrasonic.h"
#include "servo.h"
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>

// Define shared variables declared in motors.h
const double original_speed = 0.30;
// distance_threshold originally 20.0 cm -> convert to inches
const double distance_threshold = (20.0 / 2.54);
double percent_speed = 0.30;

// void disable_motors(void)
// {
//     // Configure motor pins as outputs
//     DDRD &= ~(1 << LEFT_CONTROL);   // PD2
//     DDRD &= ~(1 << LEFT_PWM);       // PD5
//     DDRD &= ~(1 << RIGHT_CONTROL);  // PD4
//     DDRD &= ~(1 << RIGHT_PWM);      // PD6
// }

void enable_motors(void)
{
    // Configure motor pins as outputs
    DDRD |= (1 << LEFT_CONTROL);   // PD2
    DDRD |= (1 << LEFT_PWM);       // PD5
    DDRD |= (1 << RIGHT_CONTROL);  // PD4
    DDRD |= (1 << RIGHT_PWM);      // PD6
}

void setup_motors(void)
{
    enable_motors();

    TCCR0A = (1 << WGM01) | (1 << WGM00);
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

    TCCR0B = (1 << CS01) | (1 << CS00);

    // Initialize PWM duty cycles to 0
    OCR0A = 0; // Right motor (PD6)
    OCR0B = 0; // Left motor (PD5)
}

void set_motor_speed(int motor, double speed, int direction)
{
    // Clamp speed to [0.0, 1.0]
    if (speed < 0.0) speed = 0.0;
    if (speed > 1.0) speed = 1.0;

    uint8_t pwm_value = (uint8_t)(speed * MOTOR_TOP_SPEED);
    if (pwm_value > MOTOR_TOP_SPEED) {
        pwm_value = MOTOR_TOP_SPEED; // Clamp to configured max
    }

    if (motor == MOTOR_LEFT) {
        OCR0B = pwm_value; // PD5
        if (direction == FORWARD) {
            PORTD |= (1 << LEFT_CONTROL);   // PD2 = HIGH => forward
        } else {
            PORTD &= ~(1 << LEFT_CONTROL);  // PD2 = LOW => backward
        }
    }
    else if (motor == MOTOR_RIGHT) {
        OCR0A = pwm_value; // PD6
        if (direction == FORWARD) {
            PORTD &= ~(1 << RIGHT_CONTROL); // PD4 = LOW => forward
        } else {
            PORTD |= (1 << RIGHT_CONTROL);  // PD4 = HIGH => backward
        }
    }
}

void follow_wall(double power_percentage, double maintain_distance, int side)
{
    // Use the caller-specified power_percentage. Do not overwrite it here.
    move_servo(side);
    _delay_ms(80);

    // Single measurement and simple clamp
    (void)measure_distance();
    _delay_ms(30);

    double distance = measure_distance();
    // Ignore wildly large readings (>30 cm -> ~11.81 in)
    if (distance <= 0.0 || distance > (30.0 / 2.54)) {
        distance = maintain_distance;
    }

    double left_speed, right_speed;
    if (side == MOTOR_RIGHT) {
        left_speed  = power_percentage * (distance / maintain_distance);
        right_speed = power_percentage * (maintain_distance / distance);
    }
    else { // MOTOR_LEFT
        left_speed  = power_percentage * (distance / maintain_distance);
        right_speed = power_percentage * (maintain_distance / distance);
    }

    // Clamp speeds
    if (left_speed > 1.0) left_speed = 1.0;
    if (right_speed > 1.0) right_speed = 1.0;

    set_motor_speed(MOTOR_LEFT,  left_speed,  FORWARD);
    set_motor_speed(MOTOR_RIGHT, right_speed, FORWARD);

    _delay_ms(40);
}
