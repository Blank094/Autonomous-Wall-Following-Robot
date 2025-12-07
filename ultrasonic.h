#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <avr/io.h>

#define TRIGGER_PIN PB4
#define ECHO_PIN    PB5

void setup_ultrasonic(void);
double measure_distance(void);       // Single optimized measurement
double measure_distance_fast(void);  // Alias for measure_distance
double measure_distance_avg(void);   // Averaged 2-sample measurement
double proximity_read(int direction);
void debug_ultrasonic(void);

#endif
