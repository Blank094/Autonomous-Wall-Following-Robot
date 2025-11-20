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

void setup_ultrasonic(void) {
    // Pin setup
    DDRB |= (1 << TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PORTB &= ~(1 << TRIGGER_PIN);

    // Do NOT configure Timer2 here; time.c configures Timer2 for system timekeeping.

    // Pin change interrupt setup for echo pin
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << ECHO_PIN);
}

ISR(PCINT0_vect) {
    if (PINB & (1 << ECHO_PIN)) {
        // Rising edge - start of echo
        echo_start_us = micros();
    } else {
        // Falling edge - end of echo
        echo_end_us = micros();
        echo_ready = 1;
    }
}

double measure_distance(void) {
    // Trigger pulse
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);

    // wait for echo or timeout
    uint32_t start = micros();
    echo_ready = 0;
    const uint32_t timeout_us = 30000UL; // 30 ms max
    while (!echo_ready) {
        if ((micros() - start) > timeout_us) {
            return 0.0; // timeout / out of range
        }
    }

    uint32_t duration_us = echo_end_us - echo_start_us;

    // Distance in cm = (time_us * 0.0343) / 2
    // Convert to inches: 1 in = 2.54 cm
    double distance_cm = (duration_us * 0.0343) / 2.0;
    double distance_in = distance_cm / 2.54;

    // Max range ~ 240 cm -> ~94.5 in
    if (distance_in > 94.5) {
        return 0.0;
    }
    return distance_in;
}

double proximity_read(int direction) {
    move_servo(direction);
    _delay_ms(200);

    // flush reading
    (void)measure_distance();
    _delay_ms(60);

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
