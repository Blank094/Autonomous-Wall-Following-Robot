#include "ultrasonic.h"
#include "servo.h"
#include "uart.h"
#include "time.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

// Use micros() timestamps (from time.c) and PCINT to time the echo pulse.
static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_end_us = 0;
static volatile uint8_t  echo_ready = 0;
static volatile uint8_t  echo_started = 0;

// Pre-calculated conversion factor: (0.0343 / 2) / 2.54 = 0.00675197
#define US_TO_INCHES 0.00675197

void setup_ultrasonic(void) {
    // Pin setup
    DDRB |= (1 << TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIGGER_PIN);

    // Pin change interrupt setup for echo pin
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << ECHO_PIN);
}

// Optimized ISR - minimal operations for fastest response
ISR(PCINT0_vect) {
    uint32_t now = micros();  // Capture time immediately
    if (PINB & (1 << ECHO_PIN)) {
        // Rising edge - start of echo
        echo_start_us = now;
        echo_started = 1;
    } else if (echo_started) {
        // Falling edge - end of echo (only if we saw rising edge)
        echo_end_us = now;
        echo_ready = 1;
        echo_started = 0;
    }
}

// Fast single measurement - optimized for speed
double measure_distance(void) {
    // Reset state
    echo_ready = 0;
    echo_started = 0;
    
    // Trigger pulse - minimum 10us required
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);

    // Wait for echo with short timeout (15ms = ~255 inches max, plenty for 6-10 inch target)
    uint32_t start = micros();
    const uint32_t timeout_us = 15000UL;  // Reduced timeout for faster failure detection
    
    while (!echo_ready) {
        if ((micros() - start) > timeout_us) {
            return 0.0; // timeout / out of range
        }
    }

    // Calculate distance directly in inches
    uint32_t duration_us = echo_end_us - echo_start_us;
    double distance_in = duration_us * US_TO_INCHES;

    // Validate range (1.5 to 80 inches useful range)
    if (distance_in < 1.5 || distance_in > 80.0) {
        return 0.0;
    }
    return distance_in;
}

// Fast measurement with minimal delay - use when servo is already positioned
double measure_distance_fast(void) {
    return measure_distance();  // Already optimized
}

// Take averaged reading (2 samples) for more accuracy when needed
double measure_distance_avg(void) {
    double d1 = measure_distance();
    _delay_us(500);  // Minimal settling time
    double d2 = measure_distance();
    
    if (d1 == 0.0) return d2;
    if (d2 == 0.0) return d1;
    
    // Return average if both valid
    return (d1 + d2) / 2.0;
}

double proximity_read(int direction) {
    move_servo(direction);
    _delay_ms(80);  // Reduced servo settling time

    // Single flush + measure for speed
    (void)measure_distance();
    _delay_ms(20);

    return measure_distance();
}

/* Ultrasonic Debugging */

void debug_ultrasonic(void) {
    char buffer[64];

    // Trigger a measurement
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);

    _delay_ms(30);

    // Measure the distance in inches
    double test_distance = measure_distance();
    uint32_t distance_inch = (uint32_t)(test_distance * 100);

    // Format and print the distance in inches
    snprintf(buffer, sizeof(buffer), "Raw Distance: %lu.%02lu in",
             distance_inch / 100, distance_inch % 100);
    uart_println(buffer);
}
