#include "servo.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

// Track current servo position to avoid unnecessary moves
static volatile int current_position = SERVO_FRONT;
static volatile uint8_t servo_moving = 0;

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
    current_position = SERVO_FRONT;
}

// Check if servo is already at target position
uint8_t servo_at_position(int pulseWidth)
{
    // Allow small tolerance (±5) for position matching
    int diff = current_position - pulseWidth;
    if (diff < 0) diff = -diff;
    return (diff <= 5);
}

// Get current servo position
int get_servo_position(void)
{
    return current_position;
}

// Move servo only if not already at position
// Returns 1 if move was needed, 0 if already at position
uint8_t move_servo(int pulseWidth)
{
    // Clamp pulse width to allowed servo range
    if (pulseWidth < SERVO_RIGHT) pulseWidth = SERVO_RIGHT;
    if (pulseWidth > SERVO_LEFT)  pulseWidth = SERVO_LEFT;
    
    // Skip if already at position
    if (servo_at_position(pulseWidth)) {
        return 0;  // No move needed
    }
    
    OCR1B = pulseWidth;
    current_position = pulseWidth;
    return 1;  // Move initiated
}

// Move servo with calculated delay based on distance
// Returns suggested wait time in ms (0 if no move needed)
uint8_t move_servo_smart(int pulseWidth, uint8_t *wait_ms)
{
    // Clamp pulse width
    if (pulseWidth < SERVO_RIGHT) pulseWidth = SERVO_RIGHT;
    if (pulseWidth > SERVO_LEFT)  pulseWidth = SERVO_LEFT;
    
    // Check if already at position
    if (servo_at_position(pulseWidth)) {
        *wait_ms = 0;
        return 0;
    }
    
    // Calculate distance to move (in pulse width units)
    int distance = current_position - pulseWidth;
    if (distance < 0) distance = -distance;
    
    // Estimate wait time: ~0.15ms per unit of pulse width change
    // Full range (150-625 = 475 units) takes ~70ms for fast servos
    // Scale: distance * 70 / 475 ≈ distance * 0.15
    uint8_t estimated_wait = (uint8_t)((distance * 15) / 100);
    if (estimated_wait < 20) estimated_wait = 20;  // Minimum 20ms
    if (estimated_wait > 80) estimated_wait = 80;  // Maximum 80ms
    
    *wait_ms = estimated_wait;
    OCR1B = pulseWidth;
    current_position = pulseWidth;
    return 1;
}