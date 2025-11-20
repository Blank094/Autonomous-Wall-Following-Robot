#include "servo.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>   // for printf if needed

void setup_servo(void)
{
    // SERVO_PIN pin as output
    DDRB |= (1 << SERVO_PIN);

    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1);
    TCCR1B = (1 << WGM13) | (1 << WGM12);

    // Prescaler 64 => ~50Hz for servo if OCR1A = 4999
    TCCR1B |= (1 << CS11) | (1 << CS10);

    // Period = 20ms -> (16MHz / 64) = 250kHz => 20ms = 5000 counts
    OCR1A = 4999; // 20ms period
    OCR1B = SERVO_FRONT; // initial servo position
}

void move_servo(int pulseWidth)
{
    // Clamp pulse width to allowed servo range
    if (pulseWidth < SERVO_RIGHT) pulseWidth = SERVO_RIGHT;
    if (pulseWidth > SERVO_LEFT)  pulseWidth = SERVO_LEFT;
    OCR1B = pulseWidth;
}