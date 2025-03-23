#ifndef ELEVATORCONTROLLER_H
#define ELEVATORCONTROLLER_H

#include "events.h"

// States of the elevator system
typedef enum {
    OFF,
    INIT,
    FLOOR2,
    FLOOR3,
    FLOOR4,
    GOINGUPTO3,
    GOINGDNTO3,
    GOINGUPTO4,
    GOINGDNTO2
} elevatorStateEnum;

void controller_tick();
void controller_init();

const char *elevatorStateEnumNames(elevatorStateEnum e);

// Visibility to support testing
elevatorStateEnum transition(elevatorStateEnum state, eventEnum event);

// Getter functions for testing
elevatorStateEnum get_current_state();
unsigned int get_timer();

#endif