#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        TIMER_EXPIRED,
        POWER_ON,
        DOOR_IS_OPEN,
        DOOR_IS_CLOSED,
        DOOR_IS_OBSTRUCTED,
        CAB_POSITION_FLOOR_2,
        CAB_POSITION_FLOOR_2_5,
        CAB_POSITION_FLOOR_3,
        CAB_POSITION_FLOOR_3_5,
        CAB_POSITION_FLOOR_4,
        CALL_FLOOR_2,
        CALL_FLOOR_3,
        CALL_FLOOR_4,
        REQ_DOOR_OPEN,
        REQ_STOP,
        REQ_FLOOR_2,
        REQ_FLOOR_3,
        REQ_FLOOR_4,
        REQ_BELL_PRESSED,
        REQ_BELL_RELEASED
    } eventEnum;

    void event_to_controller(eventEnum e);

    const char *eventEnumName(eventEnum e);

    /*

    Elevator Controls are sent to the elevator in a single 32 bit word
    with the apropriate bits set.

    You must provide all of the controls, each time the command is sent.
    Commands that contradict, like GO_UP and GO_DOWN set at the same time
    will cause an error.

    */

#define ALL_OFF 0x000000000
#define GO_UP 0x00000001
#define GO_DOWN 0x00000002
#define STOP 0x00000004
#define OPEN_DOOR 0x0000008
#define CLOSE_DOOR 0x00000010
#define STOP_DOOR 0x00000020

    int elevator_control_cmd(unsigned int);

    /*

    Indicators are display devices on either the elevator cab or on the panels
    installed on each floor.

    They are like commands, in that you send a single 32 bit word, and the bits set
    define what is turned on.

    */

#define CALL_ACCEPTED_FLOOR_2 0x00000001
#define CALL_ACCEPTED_FLOOR_3 0x00000002
#define CALL_ACCEPTED_FLOOR_4 0x00000004
#define UPPTAGEN_FLOOR_2 0x00000008
#define UPPTAGEN_FLOOR_3 0x00000010
#define UPPTAGEN_FLOOR_4 0x00000020
#define REQ_FLOOR_ACCEPTED_2 0x00000040
#define REQ_FLOOR_ACCEPTED_3 0x00000080
#define REQ_FLOOR_ACCEPTED_4 0x00000100
#define CAB_POS_2 0x00000200
#define CAB_POS_3 0x00000400
#define CAB_POS_4 0x00000800
#define POS_FLOOR_2 0x00001000
#define POS_FLOOR_3 0x00002000
#define POS_FLOOR_4 0x00004000
#define RING_BELL 0x00008000

    int elevator_indicators(unsigned int);
    unsigned int indicators();

#ifdef __cplusplus
}
#endif
