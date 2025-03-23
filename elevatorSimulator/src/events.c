#include <assert.h>
#include <string.h>

#include "events.h"

// This function converts an enumeration representing an
// event into a char*.  It is intended for printing out
// events for debugging in a human readable manner.

const char *eventEnumName(eventEnum e)
{
    const char *n[] = {
        "TIMER_EXPIRED",
        "POWER_ON",
        "DOOR_IS_OPEN",
        "DOOR_IS_CLOSED",
        "DOOR_IS_OBSTRUCTED",
        "CAB_POSITION_FLOOR_2",
        "CAB_POSITION_FLOOR_2_5",
        "CAB_POSITION_FLOOR_3",
        "CAB_POSITION_FLOOR_3_5",
        "CAB_POSITION_FLOOR_4",
        "CALL_FLOOR_2",
        "CALL_FLOOR_3",
        "CALL_FLOOR_4",
        "REQ_DOOR_OPEN",
        "REQ_STOP",
        "REQ_FLOOR_2",
        "REQ_FLOOR_3",
        "REQ_FLOOR_4",
        "REQ_BELL_PRESSED",
        "REQ_BELL_RELEASED"};

    // This code is fragile.  New events may not be reflected
    // in this function, so the below asserts are an attempt
    // to identify if there is a problem.
    assert(e >= TIMER_EXPIRED && e <= REQ_BELL_RELEASED);
    assert(!strcmp(n[REQ_BELL_RELEASED], "REQ_BELL_RELEASED"));
    if (e<(REQ_BELL_RELEASED+1))
            {
            return n[e];
            }
    return "error";
}
