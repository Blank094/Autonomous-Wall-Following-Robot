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

// Distance constraints (converted from friend's cm values)
#define DESIRED_DISTANCE_IN  (25.4 / CM_TO_IN)   // ~10 in
#define TOO_CLOSE_IN         (15.24 / CM_TO_IN)  // ~6 in
#define OBSTACLE_TH_IN       (25.4 / CM_TO_IN)   // ~10 in
#define WALL_LOST_TH_IN      (38.1 / CM_TO_IN)   // ~15 in

// Proportional speed constraints (percent of full speed)
#define BASE_PERCENT     0.30
#define MAX_PERCENT      0.60
#define MIN_PERCENT      0.15
#define TURN_PERCENT     0.35

#define SERVO_STABILIZE_MS  200
#define MEASUREMENT_SAMPLES 3
#define SAMPLE_DELAY_MS     25

// Loop delays
#define TURN_DURATION_MS    300
#define CORRECTION_MS       200
#define LOOP_DELAY_MS       100
#define MOVEMENT_DURATION_MS 200
#define CORRECTION_MOVE_MS  150
// Fast run settings (after checks)
#define FAST_PERCENT_MULT    1.6
#define FAST_MOVE_DURATION_MS 500

// Robot states
typedef enum {
    STATE_CHECK_FRONT,
    STATE_CHECK_RIGHT,
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

// Averaged ultrasonic measurement (returns inches)
static double take_precise_measurement_in(void) {
    double total = 0.0;
    uint8_t valid = 0;
    for (uint8_t i = 0; i < MEASUREMENT_SAMPLES; i++) {
        double d = measure_distance(); // already returns inches in this project
        if (d > 0.0 && d < (400.0 / CM_TO_IN)) { total += d; valid++; }
        _delay_ms(SAMPLE_DELAY_MS);
    }
    if (valid == 0) return (100.0 / CM_TO_IN);
    return total / (double)valid;
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
    double adjust = smoothed * 0.03; // tuned factor
    if (adjust > 0.10) adjust = 0.10;
    if (adjust < -0.10) adjust = -0.10;

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

    motor_stop_all();
    _delay_ms(500);

    for (;;) {
        switch (state) {
            case STATE_CHECK_FRONT:
                motor_stop_all();
                move_servo(CENTER_POS);
                _delay_ms(SERVO_STABILIZE_MS);
                front_distance = take_precise_measurement_in();
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
                right_distance = take_precise_measurement_in();
                move_servo(CENTER_POS);
                _delay_ms(SERVO_STABILIZE_MS / 2);

                {
                    char b[64];
                    snprintf(b, sizeof(b), "Right: %.2fin", right_distance);
                    uart_println(b);
                }

                if (right_distance > 0.0 && right_distance < TOO_CLOSE_IN) {
                    uart_println("Action: correct_from_wall");
                    correct_from_wall();
                    state = STATE_CHECK_FRONT;
                } else if (right_distance > WALL_LOST_TH_IN) {
                    uart_println("Action: find_wall");
                    find_wall();
                    state = STATE_CHECK_FRONT;
                } else {
                    // follow for a short burst and then re-evaluate front
                    // use fast run mode after checks to cover ground quicker
                    follow_wall_behavior(right_distance, true);
                    state = STATE_CHECK_FRONT;
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
