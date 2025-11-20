#ifndef MANEUVERS_H
#define MANEUVERS_H

#include "motors.h"
#include <util/delay.h>

void no_more_wall(void);
void maneuver_toward_wall(void);
void maneuver_small_distance(void);
void maneuver_left(void);

#endif