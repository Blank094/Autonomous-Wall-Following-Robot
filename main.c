/*
 * main.c
 * Adapted wall-following state machine using this project's motor/servo/ultrasonic APIs.
 * Behavior inspired by friend's working code. Distances are in inches.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "maneuvers.h"
#include "motors.h"
#include "ultrasonic.h"
#include "servo.h"
#include "time.h"
#include "uart.h"

// Servo positions (pulse widths defined in servo.h)
#define RIGHT_POS   SERVO_RIGHT
#define CENTER_POS  SERVO_FRONT

// Convert friend's cm constants to inches because this project uses inches
#define CM_TO_IN    2.54

// Distance constraints (in inches)
// Desired distance from wall: 6 in
#define DESIRED_DISTANCE_IN  5.0
// Too close threshold: 3 in
#define TOO_CLOSE_IN         3.0
// Obstacle threshold in front (unused separate value, set equal to desired)
#define OBSTACLE_TH_IN       8.0
// Wall lost threshold: 12 in
#define WALL_LOST_TH_IN      12.0
// Proportional speed constraints (percent of full speed)
#define BASE_PERCENT     0.20
#define MAX_PERCENT      0.35
#define MIN_PERCENT      0.12
#define TURN_PERCENT     0.25

#define SERVO_STABILIZE_MS  150  // Reduced from 200ms for faster operation
#define MEASUREMENT_SAMPLES 3    // Reduced from 5 for faster measurements
#define SAMPLE_DELAY_MS     10   // Reduced from 15ms for faster sampling

// Measurement optimization parameters
#define OUTLIER_THRESHOLD_IN 5.0
#define MIN_VALID_DISTANCE_IN 2.0
#define MAX_VALID_DISTANCE_IN 100.0

// Moving average filter
#define FILTER_WINDOW 3

// No-reading retry parameters
#define MAX_NO_READING_RETRIES 3
#define RETRY_DELAY_MS 50

// Loop delays
#define TURN_DURATION_MS    300   
#define CORRECTION_MS       200
#define LOOP_DELAY_MS       100
#define MOVEMENT_DURATION_MS 200
#define CORRECTION_MOVE_MS  150
// Fast run settings (after checks) - Disabled for smoother control
#define FAST_PERCENT_MULT    1.3
#define FAST_MOVE_DURATION_MS 500

// Robot states
typedef enum {
    STATE_CHECK_FRONT,
    STATE_CHECK_RIGHT,
    STATE_FORCE_WALL_CHECK,  // New state for when readings fail
    STATE_NORMAL
} robot_state_t;

// Helpers: motor wrappers using existing set_motor_speed API
static void motor_stop_all(void) {
    set_motor_speed(MOTOR_LEFT,  0.0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0.0, FORWARD);
}

static void motor_move_forward_pct(double left_pct, double right_pct) {
    if (left_pct < 0.0) left_pct = 0.0; if (left_pct > 1.0) left_pct = 1.0;
    if (right_pct < 0.0) right_pct = 0.0; if (right_pct > 1.0) right_pct = 1.0;
    set_motor_speed(MOTOR_LEFT, left_pct, FORWARD);
    set_motor_speed(MOTOR_RIGHT, right_pct, FORWARD);
}

static void motor_turn_left_pct(double pct) {
    if (pct < 0.0) pct = 0.0; if (pct > 1.0) pct = 1.0;
    set_motor_speed(MOTOR_LEFT, pct, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, pct, FORWARD);
}

static void motor_turn_right_pct(double pct) {
    if (pct < 0.0) pct = 0.0; if (pct > 1.0) pct = 1.0;
    set_motor_speed(MOTOR_LEFT, pct, FORWARD);
    set_motor_speed(MOTOR_RIGHT, pct, BACKWARD);
}

// Moving average filter state
static double right_distance_history[FILTER_WINDOW] = {0};
static uint8_t history_index = 0;
static uint8_t no_reading_count = 0;  // Track consecutive failed readings

// Fast measurement - takes single sample with basic validation
static double take_fast_measurement_in(void) {
    double d = measure_distance();
    if (d >= MIN_VALID_DISTANCE_IN && d <= MAX_VALID_DISTANCE_IN) {
        return d;
    }
    return 0.0;
}

// Optimized measurement with median filter
static double take_precise_measurement_in(void) {
    double samples[MEASUREMENT_SAMPLES];
    uint8_t valid_count = 0;
    
    // Collect samples
    for (uint8_t i = 0; i < MEASUREMENT_SAMPLES; i++) {
        double d = measure_distance(); // Already returns inches
        
        // Validate sample range
        if (d >= MIN_VALID_DISTANCE_IN && d <= MAX_VALID_DISTANCE_IN) {
            samples[valid_count++] = d;
            
            #ifdef DEBUG_SAMPLES
            char buf[32];
            snprintf(buf, sizeof(buf), " sample %d: %.2fin", i, d);
            uart_println(buf);
            #endif
        }
        
        if (i < MEASUREMENT_SAMPLES - 1) {
            _delay_ms(SAMPLE_DELAY_MS);
        }
    }
    
    // If no valid samples, return 0 (timeout/error)
    if (valid_count == 0) {
        return 0.0;
    }
    
    // If only 1-2 samples, return average
    if (valid_count <= 2) {
        double sum = 0.0;
        for (uint8_t i = 0; i < valid_count; i++) {
            sum += samples[i];
        }
        return sum / valid_count;
    }
    
    // Apply median filter (more robust to outliers than average)
    // Simple bubble sort for small arrays
    for (uint8_t i = 0; i < valid_count - 1; i++) {
        for (uint8_t j = 0; j < valid_count - i - 1; j++) {
            if (samples[j] > samples[j + 1]) {
                double temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    
    // Return median value
    double median;
    if (valid_count % 2 == 0) {
        median = (samples[valid_count/2 - 1] + samples[valid_count/2]) / 2.0;
    } else {
        median = samples[valid_count/2];
    }
    
    return median;
}

// Apply moving average filter to smooth rapid changes
static double apply_moving_average(double new_value) {
    if (new_value == 0.0) {
        return new_value; // Don't filter timeout/error values
    }
    
    // Update history
    right_distance_history[history_index] = new_value;
    history_index = (history_index + 1) % FILTER_WINDOW;
    
    // Calculate average of valid history
    double sum = 0.0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
        if (right_distance_history[i] > 0.0) {
            sum += right_distance_history[i];
            count++;
        }
    }
    
    return (count > 0) ? (sum / count) : new_value;
}

// Detect if reading changed significantly
static bool distance_changed_significantly(double old_val, double new_val) {
    if (old_val == 0.0 || new_val == 0.0) return true; // Always react to timeout
    double change = (new_val > old_val) ? (new_val - old_val) : (old_val - new_val);
    return (change > 1.0); // 1 inch threshold
}

// Wall-following smoothing state
static double previous_error = 0.0;
static uint16_t movement_duration_ms = MOVEMENT_DURATION_MS;

static void follow_wall_behavior(double right_distance_in, bool fast) {
    // error: positive means we're too far from wall (need to steer right),
    // negative means too close (steer left)
    double error = right_distance_in - DESIRED_DISTANCE_IN;
    double smoothed = 0.5 * error + 0.5 * previous_error;
    previous_error = smoothed;

    // scale into a small percent adjustment
    double adjust = smoothed * 0.02; // reduced tuning factor for smoother steering
    if (adjust > 0.08) adjust = 0.08;
    if (adjust < -0.08) adjust = -0.08;

    double base = BASE_PERCENT;
    if (fast) {
        base = BASE_PERCENT * FAST_PERCENT_MULT;
    }
    if (base > MAX_PERCENT) base = MAX_PERCENT;

    double left_pct  = base - adjust;
    double right_pct = base + adjust;

    if (left_pct < MIN_PERCENT) left_pct = MIN_PERCENT;
    if (right_pct < MIN_PERCENT) right_pct = MIN_PERCENT;
    if (left_pct > MAX_PERCENT) left_pct = MAX_PERCENT;
    if (right_pct > MAX_PERCENT) right_pct = MAX_PERCENT;

    motor_move_forward_pct(left_pct, right_pct);
    uint16_t dur = fast ? FAST_MOVE_DURATION_MS : movement_duration_ms;
    for (uint16_t t = 0; t < dur; t++) _delay_ms(1);
    motor_stop_all();
}

static void correct_from_wall(void) {
    // gentle left with slight speed difference
    motor_turn_left_pct(0.0);
    motor_move_forward_pct(MIN_PERCENT, MIN_PERCENT + 0.05);
    for (uint16_t t = 0; t < CORRECTION_MOVE_MS; t++) _delay_ms(1);
    motor_stop_all();
}

static void avoid_obstacle(void) {
    motor_turn_left_pct(TURN_PERCENT);
    for (uint16_t t = 0; t < TURN_DURATION_MS; t++) _delay_ms(1);
    motor_stop_all();
    _delay_ms(50);
    motor_move_forward_pct(MIN_PERCENT, MIN_PERCENT);
    for (uint16_t t = 0; t < CORRECTION_MOVE_MS; t++) _delay_ms(1);
    motor_stop_all();
}

static void find_wall(void) {
    motor_turn_right_pct(MIN_PERCENT + 0.05);
    for (uint16_t t = 0; t < CORRECTION_MOVE_MS; t++) _delay_ms(1);
    motor_stop_all();
    _delay_ms(50);
    motor_move_forward_pct(MIN_PERCENT, MIN_PERCENT);
    for (uint16_t t = 0; t < CORRECTION_MOVE_MS; t++) _delay_ms(1);
    motor_stop_all();
    _delay_ms(100);
    motor_move_forward_pct(MIN_PERCENT + 0.10, MIN_PERCENT + 0.10);
    for (uint16_t t = 0; t < 200; t++) _delay_ms(1);
    motor_stop_all();
}

static void force_wall_recheck(void) {
    uart_println("Force recheck: No valid readings detected");
    // Stop and reposition
    motor_stop_all();
    _delay_ms(100);
    
    // Small turn right to potentially find wall
    motor_turn_right_pct(MIN_PERCENT);
    for (uint16_t t = 0; t < 150; t++) _delay_ms(1);
    motor_stop_all();
    _delay_ms(100);
    
    // Move forward slightly
    motor_move_forward_pct(MIN_PERCENT, MIN_PERCENT);
    for (uint16_t t = 0; t < 100; t++) _delay_ms(1);
    motor_stop_all();
}

int main(void) {
    // Init hardware
    setup_motors();
    setup_ultrasonic();
    setup_servo();
    init_time();
    uart_init(9600);

    // Center servo and let settle
    move_servo(CENTER_POS);
    _delay_ms(1000);

    sei();

    robot_state_t state = STATE_CHECK_FRONT;
    double front_distance = 0.0;
    double right_distance = 0.0;
    double prev_right_distance = 0.0;
    uint8_t force_check_retries = 0;

    motor_stop_all();
    _delay_ms(500);

    for (;;) {
        switch (state) {
            case STATE_CHECK_FRONT:
                motor_stop_all();
                move_servo(CENTER_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                
                // Use fast measurement for front check
                front_distance = take_fast_measurement_in();
                
                {
                    char b[64];
                    snprintf(b, sizeof(b), "Front: %.2fin", front_distance);
                    uart_println(b);
                }
                
                if (front_distance < OBSTACLE_TH_IN && front_distance > 0.0) {
                    uart_println("Action: avoid_obstacle");
                    avoid_obstacle();
                    state = STATE_CHECK_FRONT;
                } else {
                    state = STATE_CHECK_RIGHT;
                }
                break;

            case STATE_CHECK_RIGHT:
                move_servo(RIGHT_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                
                // Take optimized measurement with median filter (faster now with 3 samples)
                double raw_right = take_precise_measurement_in();
                
                // Check if we got a valid reading
                if (raw_right == 0.0) {
                    no_reading_count++;
                    uart_println("No valid right reading");
                    
                    if (no_reading_count >= MAX_NO_READING_RETRIES) {
                        uart_println("Too many failed readings, forcing wall check");
                        state = STATE_FORCE_WALL_CHECK;
                        force_check_retries = 0;
                        no_reading_count = 0;
                        break;
                    } else {
                        // Quick retry
                        _delay_ms(RETRY_DELAY_MS);
                        raw_right = take_fast_measurement_in();
                        if (raw_right == 0.0) {
                            // Still no reading, try to find wall
                            find_wall();
                            state = STATE_CHECK_FRONT;
                            break;
                        }
                    }
                } else {
                    // Reset counter on successful reading
                    no_reading_count = 0;
                }
                
                // Apply moving average for temporal smoothing
                right_distance = apply_moving_average(raw_right);
                
                move_servo(CENTER_POS);
                _delay_ms(SERVO_STABILIZE_MS / 2);

                {
                    char b[64];
                    snprintf(b, sizeof(b), "Right: %.2fin (raw: %.2fin)", 
                             right_distance, raw_right);
                    uart_println(b);
                }

                // Only react if distance changed significantly or is critical
                bool critical = (right_distance > 0.0 && right_distance < TOO_CLOSE_IN) ||
                                (right_distance > WALL_LOST_TH_IN);
                
                if (critical || distance_changed_significantly(prev_right_distance, right_distance)) {
                    if (right_distance > 0.0 && right_distance < TOO_CLOSE_IN) {
                        uart_println("Action: correct_from_wall");
                        correct_from_wall();
                        state = STATE_CHECK_FRONT;
                    } else if (right_distance > WALL_LOST_TH_IN) {
                        uart_println("Action: find_wall");
                        find_wall();
                        state = STATE_CHECK_FRONT;
                    } else {
                        follow_wall_behavior(right_distance, true);
                        state = STATE_CHECK_FRONT;
                    }
                    
                    prev_right_distance = right_distance;
                } else {
                    // Distance hasn't changed significantly, keep following
                    uart_println("Action: continue (no significant change)");
                    follow_wall_behavior(right_distance, true);
                    state = STATE_CHECK_FRONT;
                }
                break;

            case STATE_FORCE_WALL_CHECK:
                // Dedicated state for recovering from lost wall readings
                force_wall_recheck();
                
                // Try to get a reading after repositioning
                move_servo(RIGHT_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                double test_reading = take_fast_measurement_in();
                
                if (test_reading > 0.0) {
                    uart_println("Wall reacquired!");
                    right_distance = test_reading;
                    prev_right_distance = right_distance;
                    no_reading_count = 0;
                    state = STATE_CHECK_FRONT;
                } else {
                    force_check_retries++;
                    if (force_check_retries >= 3) {
                        uart_println("Cannot reacquire wall, continuing anyway");
                        state = STATE_CHECK_FRONT;
                        force_check_retries = 0;
                    } else {
                        // Try again
                        _delay_ms(RETRY_DELAY_MS);
                        state = STATE_FORCE_WALL_CHECK;
                    }
                }
                break;

            default:
                state = STATE_CHECK_FRONT;
                break;
        }

        _delay_ms(LOOP_DELAY_MS);
    }

    return 0;
}
