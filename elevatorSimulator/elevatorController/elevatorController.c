#include <assert.h>
#include <stdio.h>
#include "debug.h"
#include "elevatorController.h"
#include "events.h"

static volatile elevatorStateEnum currentState;
static volatile unsigned int timer;

void off_entry();
void init_entry();
void floor2_state_entry();
void floor3_state_entry();
void floor4_state_entry();
void goingdnto2_state_entry();
void goingdnto3_state_entry();
void goingupto3_state_entry();
void goingupto4_state_entry();

void (*on_entry[GOINGDNTO2 + 1])() = {off_entry, init_entry, floor2_state_entry, floor3_state_entry, floor4_state_entry, goingupto3_state_entry, goingdnto3_state_entry, goingupto4_state_entry, goingdnto2_state_entry};

void (*on_exit[GOINGDNTO2 + 1])() = {NULL};

typedef struct {
    elevatorStateEnum nextState;
} stateInfo_t;

#define NONE (-1)

const stateInfo_t fsm[GOINGDNTO2 + 1][REQ_BELL_RELEASED + 1] = {
    /* OFF */ {{INIT}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* INIT */ {{FLOOR2}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* FLOOR2 */ {{NONE}, {NONE}, {GOINGUPTO3}, {GOINGUPTO4}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {GOINGUPTO3}, {GOINGUPTO4}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* FLOOR3 */ {{NONE}, {NONE}, {GOINGDNTO2}, {NONE}, {GOINGUPTO4}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {GOINGDNTO2}, {NONE}, {GOINGUPTO4}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* FLOOR4 */ {{NONE}, {NONE}, {GOINGDNTO2}, {GOINGDNTO3}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {GOINGDNTO2}, {GOINGDNTO3}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* GOINGUPTO3 */ {{NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {FLOOR3}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* GOINGDNTO3 */ {{NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {FLOOR3}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* GOINGUPTO4 */ {{NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {FLOOR4}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}},
    /* GOINGDNTO2 */ {{NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {FLOOR2}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}, {NONE}}
};

elevatorStateEnum transition(elevatorStateEnum state, eventEnum event) {
    assert(state >= OFF && state <= GOINGDNTO2);
    assert(event >= TIMER_EXPIRED && event <= REQ_BELL_RELEASED);

    elevatorStateEnum nextState = state;

    if (fsm[state][event].nextState != NONE) {
        INFO_PRINT("current state = %s\n", elevatorStateEnumNames(state));

        if (on_exit[state]) {
            assert(on_entry[state]);
            (on_exit[state])();
        }

        nextState = fsm[state][event].nextState;
        assert(nextState >= OFF && nextState <= GOINGDNTO2);

        INFO_PRINT("new state = %s\n", elevatorStateEnumNames(nextState));

        if (on_entry[nextState]) {
            assert(on_entry[nextState]);
            (on_entry[nextState])();
        }
    }

    return nextState;
}

void event_to_controller(eventEnum e) {
    INFO_PRINT("event to controller %s\n", eventEnumName(e));
    currentState = transition(currentState, e);
}

void controller_tick() {
    if (timer) {
        timer--;
        if (!timer) {
            event_to_controller(TIMER_EXPIRED);
        }
    }
}

void controller_init() {
    DEBUG_PRINT("\n");
    currentState = OFF;
    timer = 0;
}

void init_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(ALL_OFF);
    elevator_indicators(CAB_POS_2 | POS_FLOOR_2);
    timer = 20;
}

void off_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(ALL_OFF);
}

void floor2_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(STOP);
    elevator_control_cmd(OPEN_DOOR);
    elevator_indicators(CAB_POS_2 | POS_FLOOR_2);
}

void floor3_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(STOP);
    elevator_control_cmd(OPEN_DOOR);
    elevator_indicators(CAB_POS_3 | POS_FLOOR_3);
}

void floor4_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(STOP);
    elevator_control_cmd(OPEN_DOOR);
    elevator_indicators(CAB_POS_4 | POS_FLOOR_4);
}

void goingdnto2_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_DOWN);
    elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_2 | CALL_ACCEPTED_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4);
}

void goingdnto3_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_DOWN);
    elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3 | UPPTAGEN_FLOOR_4 | UPPTAGEN_FLOOR_2);
}

void goingupto3_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_UP);
    elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3 | UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_4);
}

void goingupto4_state_entry() {
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_UP);
    elevator_indicators(indicators() | REQ_FLOOR_ACCEPTED_4 | CALL_ACCEPTED_FLOOR_4 | UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3);
}
elevatorStateEnum get_current_state() {
    return currentState;
}

unsigned int get_timer() {
    return timer;
}
