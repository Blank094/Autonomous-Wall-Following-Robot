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
#define DESIRED_DISTANCE_IN  9.0
#define TOO_CLOSE_IN         4.0
#define OBSTACLE_TH_IN       14.0
#define WALL_LOST_TH_IN      18.0
#define DISTANCE_TOLERANCE   2.5

// SLOWER movement speed settings
#define CRUISE_SPEED     0.25    // Slower cruising speed
#define MAX_SPEED        0.32    // Maximum speed when perfectly aligned
#define MIN_SPEED        0.15    // Minimum speed for tight turns
#define TURN_SPEED       0.22    // Speed during obstacle avoidance turns

// PD Controller gains - tuned for slower, more precise control
#define KP               0.035  // Proportional gain
#define KD               0.015  // Derivative gain (dampens oscillation)
#define MAX_STEERING     0.15   // Maximum steering differential

#define SERVO_STABILIZE_MS  100  // Proper servo settling time
#define MEASUREMENT_SAMPLES 3    // Take 2 samples for accuracy

// Measurement optimization parameters
#define MIN_VALID_DISTANCE_IN 1.5
#define MAX_VALID_DISTANCE_IN 100.0

// Moving average filter
#define FILTER_WINDOW 3

// AGGRESSIVE reading timing
#define FRONT_CHECK_INTERVAL_MS  150  // Check front MORE often (every 150ms)
#define RIGHT_READ_DELAY_MS      20   // Small delay between right readings
#define TURN_DURATION_MS         200  // Duration for obstacle avoidance turns
#define EMERGENCY_STOP_DIST      2.0  // Emergency stop if closer than 2 inches

// Robot states for continuous movement
typedef enum {
    STATE_STARTUP,
    STATE_RUNNING,          // Normal continuous wall-following
    STATE_OBSTACLE_AVOID,   // Obstacle detected, turning left
    STATE_FIND_WALL,        // Wall lost, turning right to find it
    STATE_EMERGENCY_STOP    // Too close to something, full stop
} robot_state_t;

// Helpers: motor wrappers using existing set_motor_speed API
static void motor_stop_all(void) {
    set_motor_speed(MOTOR_LEFT,  0.0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0.0, FORWARD);
}

// Set continuous motor speeds (no stopping)
static void motor_set_speeds(double left_pct, double right_pct) {
    if (left_pct < 0.0) left_pct = 0.0; if (left_pct > 1.0) left_pct = 1.0;
    if (right_pct < 0.0) right_pct = 0.0; if (right_pct > 1.0) right_pct = 1.0;
    set_motor_speed(MOTOR_LEFT, left_pct, FORWARD);
    set_motor_speed(MOTOR_RIGHT, right_pct, FORWARD);
}

static void motor_turn_left_inplace(double pct) {
    if (pct < 0.0) pct = 0.0; if (pct > 1.0) pct = 1.0;
    set_motor_speed(MOTOR_LEFT, pct, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, pct, FORWARD);
}

// Moving average filter state
static double right_distance_history[FILTER_WINDOW] = {0};
static uint8_t history_index = 0;
static double previous_error = 0.0;  // For PD controller

// Take 2-sample measurement for accuracy
static double take_measurement(void) {
    double d1 = measure_distance();
    _delay_ms(10);  // Small delay between samples
    double d2 = measure_distance();
    
    uint8_t v1 = (d1 >= MIN_VALID_DISTANCE_IN && d1 <= MAX_VALID_DISTANCE_IN);
    uint8_t v2 = (d2 >= MIN_VALID_DISTANCE_IN && d2 <= MAX_VALID_DISTANCE_IN);
    
    if (v1 && v2) {
        return (d1 + d2) / 2.0;  // Average of both
    } else if (v1) {
        return d1;
    } else if (v2) {
        return d2;
    }
    return 0.0;
}

// Single quick measurement (for front check to save time)
static double take_quick_measurement(void) {
    double d = measure_distance();
    if (d >= MIN_VALID_DISTANCE_IN && d <= MAX_VALID_DISTANCE_IN) {
        return d;
    }
    return 0.0;
}

// Apply moving average filter to smooth readings
static double apply_filter(double new_value) {
    if (new_value == 0.0) {
        // Return last valid average on bad reading
        double sum = 0.0;
        uint8_t count = 0;
        for (uint8_t i = 0; i < FILTER_WINDOW; i++) {
            if (right_distance_history[i] > 0.0) {
                sum += right_distance_history[i];
                count++;
            }
        }
        return (count > 0) ? (sum / count) : DESIRED_DISTANCE_IN;
    }
    
    // Update history
    right_distance_history[history_index] = new_value;
    history_index = (history_index + 1) % FILTER_WINDOW;
    
    // Calculate weighted average (more weight to recent)
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

// PD Controller: Calculate steering adjustment for continuous movement
// Returns steering value: positive = steer right (toward wall), negative = steer left (away)
static double calculate_steering(double right_distance) {
    double error = right_distance - DESIRED_DISTANCE_IN;
    
    // PD control
    double derivative = error - previous_error;
    double steering = (KP * error) + (KD * derivative);
    previous_error = error;
    
    // Clamp steering
    if (steering > MAX_STEERING) steering = MAX_STEERING;
    if (steering < -MAX_STEERING) steering = -MAX_STEERING;
    
    return steering;
}

// Update motor speeds for continuous wall following
// Steering: positive = turn right toward wall, negative = turn left away
static void update_continuous_drive(double steering, double base_speed) {
    // Steering adjusts differential: positive steering = left faster, right slower = turn right
    double left_speed = base_speed + steering;
    double right_speed = base_speed - steering;
    
    // Clamp speeds
    if (left_speed < MIN_SPEED) left_speed = MIN_SPEED;
    if (right_speed < MIN_SPEED) right_speed = MIN_SPEED;
    if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
    if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
    
    motor_set_speeds(left_speed, right_speed);
}

int main(void) {
    // Init hardware
    setup_motors();
    setup_ultrasonic();
    setup_servo();
    init_time();
    uart_init(9600);

    sei();

    // Position servo to look right (wall side) - stays there during continuous operation
    move_servo(RIGHT_POS);
    _delay_ms(500);

    uart_println("Starting continuous wall-following mode");
    uart_println("Target: 6 inches from right wall");

    robot_state_t state = STATE_STARTUP;
    double right_distance = DESIRED_DISTANCE_IN;  // Assume starting at target
    double front_distance = 0.0;
    uint32_t last_front_check = 0;
    uint32_t now = 0;
    uint8_t bad_reading_count = 0;
    char debug_buf[64];  // For debug output

    motor_stop_all();
    _delay_ms(500);

    // Main continuous loop
    for (;;) {
        now = millis();
        
        switch (state) {
            case STATE_STARTUP:
                // Initial reading before starting movement
                move_servo(RIGHT_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                right_distance = apply_filter(take_measurement());
                
                snprintf(debug_buf, sizeof(debug_buf), "Startup Right: %.2fin", right_distance);
                uart_println(debug_buf);
                
                if (right_distance > 0.0) {
                    uart_println("Initial reading OK, starting...");
                    state = STATE_RUNNING;
                    last_front_check = now;
                    // Start moving at slow cruise speed
                    motor_set_speeds(CRUISE_SPEED, CRUISE_SPEED);
                } else {
                    uart_println("Waiting for valid reading...");
                    _delay_ms(100);
                }
                break;

            case STATE_RUNNING:
                // === CONTINUOUS WALL FOLLOWING WITH AGGRESSIVE READING ===
                
                // ALWAYS take right wall measurement first (servo should be pointing right)
                if (!servo_at_position(RIGHT_POS)) {
                    move_servo(RIGHT_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                }
                
                // Take aggressive 2-sample right reading
                {
                    double raw = take_measurement();
                    right_distance = apply_filter(raw);
                    
                    // Debug output for right reading
                    snprintf(debug_buf, sizeof(debug_buf), "R: %.2fin", right_distance);
                    uart_println(debug_buf);
                    
                    if (raw == 0.0) {
                        bad_reading_count++;
                        uart_println("Bad right reading!");
                        if (bad_reading_count > 3) {
                            uart_println("Wall lost - too many bad readings!");
                            state = STATE_FIND_WALL;
                            break;
                        }
                    } else {
                        bad_reading_count = 0;
                    }
                }
                
                // Check for emergency (too close to wall)
                if (right_distance > 0.0 && right_distance < EMERGENCY_STOP_DIST) {
                    uart_println("EMERGENCY: Too close to wall!");
                    state = STATE_EMERGENCY_STOP;
                    break;
                }
                
                // Check for wall lost
                if (right_distance > WALL_LOST_TH_IN) {
                    uart_println("Wall too far, seeking...");
                    state = STATE_FIND_WALL;
                    break;
                }
                
                // AGGRESSIVE front check (every 150ms)
                if ((now - last_front_check) > FRONT_CHECK_INTERVAL_MS) {
                    // Slow down while checking front
                    motor_set_speeds(MIN_SPEED, MIN_SPEED);
                    
                    // Move servo to front
                    move_servo(CENTER_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                    
                    // Take front measurement
                    front_distance = take_quick_measurement();
                    
                    // Debug output for front reading
                    snprintf(debug_buf, sizeof(debug_buf), "F: %.2fin", front_distance);
                    uart_println(debug_buf);
                    
                    // Return servo to right
                    move_servo(RIGHT_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                    
                    last_front_check = now;
                    
                    // Check for obstacle
                    if (front_distance > 0.0 && front_distance < OBSTACLE_TH_IN) {
                        snprintf(debug_buf, sizeof(debug_buf), "OBSTACLE at %.2fin!", front_distance);
                        uart_println(debug_buf);
                        state = STATE_OBSTACLE_AVOID;
                        break;
                    }
                }
                
                // Calculate and apply steering correction (PD control)
                {
                    double steering = calculate_steering(right_distance);
                    
                    // Keep speed slow and steady
                    double speed = CRUISE_SPEED;
                    double error_mag = right_distance - DESIRED_DISTANCE_IN;
                    if (error_mag < 0) error_mag = -error_mag;
                    
                    // Slightly faster only when well aligned
                    if (error_mag < DISTANCE_TOLERANCE) {
                        speed = MAX_SPEED;
                    } else if (error_mag > 3.0) {
                        speed = MIN_SPEED;  // Very slow when way off
                    }
                    
                    update_continuous_drive(steering, speed);
                }
                
                // Small delay for stability
                _delay_ms(RIGHT_READ_DELAY_MS);
                break;

            case STATE_OBSTACLE_AVOID:
                // Stop completely first
                motor_stop_all();
                _delay_ms(100);
                
                uart_println("Avoiding obstacle - turning left");
                
                // Turn left in place at slow speed
                motor_turn_left_inplace(TURN_SPEED);
                _delay_ms(TURN_DURATION_MS);
                motor_stop_all();
                _delay_ms(50);
                
                // Check front again after turn
                move_servo(CENTER_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                front_distance = take_quick_measurement();
                snprintf(debug_buf, sizeof(debug_buf), "After turn F: %.2fin", front_distance);
                uart_println(debug_buf);
                
                // Return servo to right
                move_servo(RIGHT_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                
                // If still blocked, turn more
                if (front_distance > 0.0 && front_distance < OBSTACLE_TH_IN) {
                    uart_println("Still blocked, turning more...");
                    // Stay in this state
                    break;
                }
                
                // Move forward slowly
                motor_set_speeds(CRUISE_SPEED, CRUISE_SPEED);
                _delay_ms(150);
                
                // Reset front check timer and resume
                last_front_check = millis();
                uart_println("Resuming wall follow");
                state = STATE_RUNNING;
                break;

            case STATE_FIND_WALL:
                // Ensure servo is pointing right to look for wall
                if (!servo_at_position(RIGHT_POS)) {
                    move_servo(RIGHT_POS);
                    _delay_ms(SERVO_STABILIZE_MS);
                }
                
                uart_println("Seeking wall - curving right");
                
                // Curve right slowly (left faster than right)
                motor_set_speeds(CRUISE_SPEED, MIN_SPEED);
                _delay_ms(200);
                
                // Take reading to check if we found the wall
                right_distance = apply_filter(take_measurement());
                snprintf(debug_buf, sizeof(debug_buf), "Seeking R: %.2fin", right_distance);
                uart_println(debug_buf);
                
                if (right_distance > 0.0 && right_distance < WALL_LOST_TH_IN) {
                    uart_println("Wall found!");
                    bad_reading_count = 0;
                    last_front_check = millis();  // Reset front check timer
                    state = STATE_RUNNING;
                }
                // Stay in FIND_WALL if not found yet
                break;

            case STATE_EMERGENCY_STOP:
                // Full stop immediately
                motor_stop_all();
                uart_println("EMERGENCY - backing up");
                _delay_ms(100);
                
                // Back up slowly
                set_motor_speed(MOTOR_LEFT, 0.18, BACKWARD);
                set_motor_speed(MOTOR_RIGHT, 0.18, BACKWARD);
                _delay_ms(200);
                motor_stop_all();
                _delay_ms(50);
                
                // Turn left away from wall
                uart_println("Turning away from wall");
                motor_turn_left_inplace(TURN_SPEED);
                _delay_ms(150);
                motor_stop_all();
                _delay_ms(50);
                
                // Take new reading
                right_distance = apply_filter(take_measurement());
                snprintf(debug_buf, sizeof(debug_buf), "After correction R: %.2fin", right_distance);
                uart_println(debug_buf);
                
                // Resume
                last_front_check = millis();
                state = STATE_RUNNING;
                break;

            default:
                state = STATE_STARTUP;
                break;
        }
        
        // Small delay for loop stability
        _delay_ms(10);
    }

    return 0;
}
