#include "maneuvers.h"
#include "motors.h"

// #define ALIGN_DELAY 300 // Adjust based on terrain (if slippery)

void no_more_wall(void) {

    // stop
    set_motor_speed(MOTOR_RIGHT, 0, BACKWARD);
    set_motor_speed(MOTOR_LEFT,  0, BACKWARD);
    _delay_ms(500);

    // turn Right
    set_motor_speed(MOTOR_LEFT,  percent_speed, FORWARD);
    set_motor_speed(MOTOR_RIGHT, percent_speed, BACKWARD);
    _delay_ms(300);

    // stop
    set_motor_speed(MOTOR_RIGHT, 0, BACKWARD);
    set_motor_speed(MOTOR_LEFT,  0, BACKWARD);
    _delay_ms(500);

    // forward
    set_motor_speed(MOTOR_RIGHT, percent_speed, FORWARD);
    set_motor_speed(MOTOR_LEFT,  percent_speed, FORWARD);
    _delay_ms(800);

    // stop
    set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    _delay_ms(500);
}

void maneuver_toward_wall(void) {
    // turn Right
    set_motor_speed(MOTOR_LEFT,  percent_speed, FORWARD);
    set_motor_speed(MOTOR_RIGHT, percent_speed, BACKWARD);
    _delay_ms(300);

    // stop
    set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    _delay_ms(500);

    // forward
    set_motor_speed(MOTOR_RIGHT, percent_speed, FORWARD);
    set_motor_speed(MOTOR_LEFT,  percent_speed, FORWARD);
    _delay_ms(800);

    // stop
    set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    _delay_ms(500);

    // align center
    set_motor_speed(MOTOR_LEFT,  percent_speed, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, percent_speed, FORWARD);
    _delay_ms(300); 

    // stop
    set_motor_speed(MOTOR_RIGHT, 0, BACKWARD);
    set_motor_speed(MOTOR_LEFT,  0, BACKWARD);
    _delay_ms(500);
}

void maneuver_small_distance(void) {
    // Move backward
    set_motor_speed(MOTOR_LEFT,  percent_speed, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, percent_speed, BACKWARD);
    _delay_ms(450);

    // stop
    set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    _delay_ms(50);

    // pivot right
    set_motor_speed(MOTOR_LEFT,  percent_speed, FORWARD);
    set_motor_speed(MOTOR_RIGHT, percent_speed, BACKWARD);
    _delay_ms(500);

    set_motor_speed(MOTOR_RIGHT, 0, BACKWARD);
    set_motor_speed(MOTOR_LEFT,  0, BACKWARD);
    _delay_ms(200);

    // Move backward again
    set_motor_speed(MOTOR_LEFT,  percent_speed, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, percent_speed, BACKWARD);
    _delay_ms(450);

    set_motor_speed(MOTOR_RIGHT, 0, BACKWARD);
    set_motor_speed(MOTOR_LEFT,  0, BACKWARD);
    _delay_ms(200);

    // // Another pivot left
    // set_motor_speed(MOTOR_LEFT,  percent_speed + 0.3, BACKWARD);
    // set_motor_speed(MOTOR_RIGHT, percent_speed + 0.3, FORWARD);
    // _delay_ms(300);

    // // stop
    // set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    // set_motor_speed(MOTOR_RIGHT, 0, FORWARD);

    // set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    // set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    // _delay_ms(200);
}

void maneuver_left(void) {
    // stop
    set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    _delay_ms(50);

    // rotate left
    set_motor_speed(MOTOR_LEFT,  original_speed + 0.3, BACKWARD);
    set_motor_speed(MOTOR_RIGHT, original_speed + 0.3, FORWARD);
    _delay_ms(300);

    // stop
    set_motor_speed(MOTOR_LEFT,  0, FORWARD);
    set_motor_speed(MOTOR_RIGHT, 0, FORWARD);
    // _delay_ms(100);
}