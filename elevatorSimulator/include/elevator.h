
#pragma once

// The physical elevator is simulated here.

#ifdef __cplusplus
extern "C"
{
#endif

#define ELEV_POWER_ON (1)
#define ELEV_POWER_OFF (0)

#define ELEV_CAB_POSITION_2 (20)
#define ELEV_CAB_POSITION_2_5 (25)
#define ELEV_CAB_POSITION_3 (30)
#define ELEV_CAB_POSITION_3_5 (35)
#define ELEV_CAB_POSITION_4 (40)

#define ELEV_CAB_DIRECTION_UP (1)
#define ELEV_CAB_DIRECTION_DOWN (-1)
#define ELEV_CAB_DIRECTION_NEUTRAL (0)

#define ELEV_DOOR_POSITION_OPEN (0)
#define ELEV_DOOR_POSITION_CLOSED (5)

#define ELEV_DOOR_DIRECTION_CLOSING (1)
#define ELEV_DOOR_DIRECTION_OPENING (-1)
#define ELEV_DOOR_DIRECTION_NOT_MOVING (0)

    // called once a second by FLTK.
    void elevator_tick();

    // called when the simulated power is turned off
    void init_elevator();

    // returns the time in seconds
    unsigned int timeInSeconds();

    void power_on();

    void power_off();

    int power_status();

    int cab_position();

    // returns the current value of indicators in the elevator
    unsigned int indicators();

    int door_position();

    void door_obstructed(int);

#ifdef __cplusplus
}
#endif
