// This is the controller for the elevator.
// It is implemented strictly in the C programming language,
// but is callable by a C++ program, like for example an FLTK
// window, or a unit test framework
//
/////////////////////////////////////////////////////////////
#pragma once

#include <stdbool.h>

#include "events.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // States of the elevator system

    typedef enum
    {
        OFF,
        INIT,
        FLOOR2,
        FLOOR3,
        FLOOR4,
        GOINGUPTO3,
        GOINGDNTO3,
        GOINGUPTO4,
        GOINGDNTO2,
        FLOOR2_DOOR_OPENING,
        FLOOR2_DOOR_CLOSING,
        FLOOR3_DOOR_OPENING,
        FLOOR3_DOOR_CLOSING,
        FLOOR4_DOOR_OPENING,
        FLOOR4_DOOR_CLOSING,
        NUM_STATES  // Used to define size of state arrays
    } elevatorStateEnum;

    void controller_tick();
    void controller_init();

    const char *elevatorStateEnumNames(elevatorStateEnum e);

    // Indicator and control access functions for testing
    int indicators();
    void set_door_state(bool is_open, bool is_closed);
    bool get_door_is_open();
    bool get_door_is_closed();

    // visibility to support testing
    elevatorStateEnum transition(elevatorStateEnum state, eventEnum event);
#ifdef __cpluspluss
}
#endif