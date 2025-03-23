#include <assert.h>
#include <stdio.h>

#include "debug.h"
#include "elevatorController.h"
#include "events.h"

// This is the elevator controller implementation that meets the requirements
// specified in the assignment. It implements a state machine that manages
// elevator movement, door operations, and indicator lights according to
// the specified requirements.

static volatile elevatorStateEnum currentState;
static volatile unsigned int timer;
static volatile bool door_is_open = false;
static volatile bool door_is_closed = true;
static volatile int current_indicators = 0;

// Forward declarations for state entry/exit functions
void off_entry();
void init_entry();
void floor2_state_entry();
void floor3_state_entry();
void floor4_state_entry();
void goingdnto2_state_entry();
void goingdnto3_state_entry();
void goingupto3_state_entry();
void goingupto4_state_entry();
void floor2_door_opening_entry();
void floor2_door_closing_entry();
void floor3_door_opening_entry();
void floor3_door_closing_entry();
void floor4_door_opening_entry();
void floor4_door_closing_entry();

// Exit functions
void floor_state_exit();
void moving_state_exit();

// Array of function pointers for entry actions, indexed by elevatorStateEnum
void (*on_entry[NUM_STATES])() = {
    off_entry,               // OFF
    init_entry,              // INIT
    floor2_state_entry,      // FLOOR2
    floor3_state_entry,      // FLOOR3
    floor4_state_entry,      // FLOOR4
    goingupto3_state_entry,  // GOINGUPTO3
    goingdnto3_state_entry,  // GOINGDNTO3
    goingupto4_state_entry,  // GOINGUPTO4
    goingdnto2_state_entry,  // GOINGDNTO2
    floor2_door_opening_entry, // FLOOR2_DOOR_OPENING
    floor2_door_closing_entry, // FLOOR2_DOOR_CLOSING
    floor3_door_opening_entry, // FLOOR3_DOOR_OPENING
    floor3_door_closing_entry, // FLOOR3_DOOR_CLOSING
    floor4_door_opening_entry, // FLOOR4_DOOR_OPENING
    floor4_door_closing_entry  // FLOOR4_DOOR_CLOSING
};

// Array of function pointers for exit actions, indexed by elevatorStateEnum
void (*on_exit[NUM_STATES])() = {
    NULL,               // OFF
    NULL,               // INIT
    floor_state_exit,   // FLOOR2
    floor_state_exit,   // FLOOR3
    floor_state_exit,   // FLOOR4
    moving_state_exit,  // GOINGUPTO3
    moving_state_exit,  // GOINGDNTO3
    moving_state_exit,  // GOINGUPTO4
    moving_state_exit,  // GOINGDNTO2
    NULL,               // FLOOR2_DOOR_OPENING
    NULL,               // FLOOR2_DOOR_CLOSING
    NULL,               // FLOOR3_DOOR_OPENING
    NULL,               // FLOOR3_DOOR_CLOSING
    NULL,               // FLOOR4_DOOR_OPENING
    NULL                // FLOOR4_DOOR_CLOSING
};

typedef struct
{
    elevatorStateEnum nextState;
} stateInfo_t;

#define f false
#define t true
#define NONE (-1)  // Meaning no transition
#define _
#define __
#define ___
#define ____
#define _____
#define ______
#define _______
#define ________
#define _________
#define __________
#define ___________
#define ____________
#define _____________
#define ______________
#define _______________
#define ________________
#define _________________
#define __________________
#define ___________________

// Complete FSM table with all states and events
// if NONE, then there is no transition for that event while in that state.
//                    STATE           EVENT
const stateInfo_t fsm[NUM_STATES][REQ_BELL_RELEASED + 1] = {
    /*              TIMER_EXPIRED     POWER_ON         DOOR_IS_OPEN     DOOR_IS_CLOSED   DOOR_IS_OBSTRUCTED   CAB_POSITION_FLOOR_2   CAB_POSITION_FLOOR_2_5   CAB_POSITION_FLOOR_3 CAB_POSITION_FLOOR_3_5  CAB_POSITION_FLOOR_4   CALL_FLOOR_2      CALL_FLOOR_3     CALL_FLOOR_4      REQ_DOOR_OPEN      REQ_STOP           REQ_FLOOR_2      REQ_FLOOR_3      REQ_FLOOR_4      REQ_BELL_PRESSED   REQ_BELL_RELEASED*/
    /*OFF       */ {{NONE}, ___________{INIT}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*INIT      */ {{FLOOR2}, _________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR2    */ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{GOINGUPTO3}, _{GOINGUPTO4}, __{FLOOR2_DOOR_OPENING}, _{NONE}, _______{NONE}, _______{GOINGUPTO3}, __{GOINGUPTO4}, ___{NONE}, __________{NONE}},
    /*FLOOR3    */ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{GOINGDNTO2}, _____{NONE}, _______{GOINGUPTO4}, __{FLOOR3_DOOR_OPENING}, _{NONE}, _______{GOINGDNTO2}, _{NONE}, ________{GOINGUPTO4}, ___{NONE}, __________{NONE}},
    /*FLOOR4    */ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{GOINGDNTO2}, _____{GOINGDNTO3}, _{NONE}, __________{FLOOR4_DOOR_OPENING}, _{NONE}, _______{GOINGDNTO2}, _{GOINGDNTO3}, __{NONE}, _________{NONE}, __________{NONE}},
    /*GOINGUPTO3*/ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{FLOOR3}, ____________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{FLOOR3}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*GOINGDNTO3*/ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{FLOOR3}, ____________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{FLOOR3}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*GOINGUPTO4*/ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{FLOOR4}, ____________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{FLOOR4}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*GOINGDNTO2*/ {{NONE}, ___________{NONE}, _________{NONE}, __________{NONE}, __________{NONE}, ____________{FLOOR2}, ____________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{FLOOR2}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR2_DOOR_OPENING*/ {{NONE}, ___{NONE}, _________{FLOOR2}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR2_DOOR_CLOSING*/ {{NONE}, ___{NONE}, _________{NONE}, __________{FLOOR2}, __________{FLOOR2_DOOR_OPENING}, __{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{FLOOR2_DOOR_OPENING}, _{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR3_DOOR_OPENING*/ {{NONE}, ___{NONE}, _________{FLOOR3}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR3_DOOR_CLOSING*/ {{NONE}, ___{NONE}, _________{NONE}, __________{FLOOR3}, __________{FLOOR3_DOOR_OPENING}, __{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{FLOOR3_DOOR_OPENING}, _{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR4_DOOR_OPENING*/ {{NONE}, ___{NONE}, _________{FLOOR4}, __________{NONE}, __________{NONE}, ____________{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{NONE}, ___________{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}},
    /*FLOOR4_DOOR_CLOSING*/ {{NONE}, ___{NONE}, _________{NONE}, __________{FLOOR4}, __________{FLOOR4_DOOR_OPENING}, __{NONE}, ______________{NONE}, _________________{NONE}, ______________{NONE}, _______________{NONE}, ______________{NONE}, ___________{NONE}, _______{NONE}, ___________{FLOOR4_DOOR_OPENING}, _{NONE}, _______{NONE}, _______{NONE}, ________{NONE}, _________{NONE}, __________{NONE}}
};

// Helper functions for state management
int indicators()
{
    return current_indicators;
}

void set_door_state(bool is_open, bool is_closed)
{
    door_is_open = is_open;
    door_is_closed = is_closed;
}

bool get_door_is_open()
{
    return door_is_open;
}

bool get_door_is_closed()
{
    return door_is_closed;
}

elevatorStateEnum transition(elevatorStateEnum state, eventEnum event)
{
    assert(state >= OFF && state < NUM_STATES);
    assert(event >= TIMER_EXPIRED && event <= REQ_BELL_RELEASED);

    elevatorStateEnum nextState = state;

    // Safety checks (ERR-002, ERR-003)
    if (fsm[state][event].nextState != NONE) {
        // Check for movement with open door (ERR-003)
        if ((state == FLOOR2 || state == FLOOR3 || state == FLOOR4) && 
            (fsm[state][event].nextState == GOINGUPTO3 || 
             fsm[state][event].nextState == GOINGUPTO4 ||
             fsm[state][event].nextState == GOINGDNTO3 ||
             fsm[state][event].nextState == GOINGDNTO2)) {
            
            // Ensure door is closed before allowing movement
            if (!door_is_closed) {
                INFO_PRINT("Movement requested but door not closed - ignoring (ERR-003)\n");
                return state; // Ignore transition
            }
        }
        
        // Check for door open during movement (ERR-002)
        if ((state == GOINGUPTO3 || state == GOINGUPTO4 || 
             state == GOINGDNTO3 || state == GOINGDNTO2) &&
            event == REQ_DOOR_OPEN) {
            INFO_PRINT("Door open requested during movement - ignoring (ERR-002)\n");
            return state; // Ignore door open request while moving
        }

        INFO_PRINT("current state = %s\n", elevatorStateEnumNames(state));

        // Run the exit actions
        if (on_exit[state])
        {
            (on_exit[state])();
        }

        // Determine next state.
        nextState = fsm[state][event].nextState;
        assert(nextState >= OFF && nextState < NUM_STATES);

        INFO_PRINT("new state = %s\n", elevatorStateEnumNames(nextState));

        // Run the entry actions
        if (on_entry[nextState])
        {
            (on_entry[nextState])();
        }
    }

    return nextState;
}

void event_to_controller(eventEnum e)
{
    INFO_PRINT("event to controller %s\n", eventEnumName(e));
    
    // Special handling for door state events
    if (e == DOOR_IS_OPEN) {
        set_door_state(true, false);
    } else if (e == DOOR_IS_CLOSED) {
        set_door_state(false, true);
    }
    
    // Special handling for bell
    if (e == REQ_BELL_PRESSED) {
        elevator_indicators(indicators() | RING_BELL); // REQ-004
        INFO_PRINT("Bell pressed - RING_BELL set\n");
    } else if (e == REQ_BELL_RELEASED) {
        elevator_indicators(indicators() & ~RING_BELL); // REQ-004
        INFO_PRINT("Bell released - RING_BELL cleared\n");
    }
    
    // Regular state transitions
    currentState = transition(currentState, e);
}

void controller_tick()
{
    if (timer)
    {
        timer--;
        if (!timer)
        {
            event_to_controller(TIMER_EXPIRED);
        }
    }
}

void controller_init()
{
    DEBUG_PRINT("\n");
    currentState = OFF;
    timer = 0;
    door_is_open = false;
    door_is_closed = true;
    current_indicators = 0;
}

// Implementation of state entry/exit functions
void off_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(ALL_OFF); // INIT-002
    current_indicators = 0;
}

void init_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(ALL_OFF); // INIT-002
    current_indicators = CAB_POS_2 | POS_FLOOR_2; // INIT-003
    elevator_indicators(current_indicators);
    timer = 20; // Wait for timer to expire before transitioning to FLOOR2
}

void floor2_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(STOP); // MOV-003
    elevator_control_cmd(OPEN_DOOR); // Open doors when arriving
    
    // Reset movement flags (MOV-004)
    current_indicators &= ~(UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4);
    current_indicators &= ~(CALL_ACCEPTED_FLOOR_2 | CALL_ACCEPTED_FLOOR_3 | CALL_ACCEPTED_FLOOR_4);
    current_indicators &= ~(REQ_FLOOR_ACCEPTED_2 | REQ_FLOOR_ACCEPTED_3 | REQ_FLOOR_ACCEPTED_4);
    
    // Set floor indicators (IND-001, IND-004)
    current_indicators &= ~(POS_FLOOR_3 | POS_FLOOR_4);
    current_indicators &= ~(CAB_POS_3 | CAB_POS_4);
    current_indicators |= POS_FLOOR_2 | CAB_POS_2;
    
    elevator_indicators(current_indicators);
}

void floor3_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(STOP); // MOV-003
    elevator_control_cmd(OPEN_DOOR);
    
    // Reset movement flags (MOV-004)
    current_indicators &= ~(UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4);
    current_indicators &= ~(CALL_ACCEPTED_FLOOR_2 | CALL_ACCEPTED_FLOOR_3 | CALL_ACCEPTED_FLOOR_4);
    current_indicators &= ~(REQ_FLOOR_ACCEPTED_2 | REQ_FLOOR_ACCEPTED_3 | REQ_FLOOR_ACCEPTED_4);
    
    // Set floor indicators (IND-002, IND-005)
    current_indicators &= ~(POS_FLOOR_2 | POS_FLOOR_4);
    current_indicators &= ~(CAB_POS_2 | CAB_POS_4);
    current_indicators |= POS_FLOOR_3 | CAB_POS_3;
    
    elevator_indicators(current_indicators);
}

void floor4_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(STOP); // MOV-003
    elevator_control_cmd(OPEN_DOOR);
    
    // Reset movement flags (MOV-004)
    current_indicators &= ~(UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4);
    current_indicators &= ~(CALL_ACCEPTED_FLOOR_2 | CALL_ACCEPTED_FLOOR_3 | CALL_ACCEPTED_FLOOR_4);
    current_indicators &= ~(REQ_FLOOR_ACCEPTED_2 | REQ_FLOOR_ACCEPTED_3 | REQ_FLOOR_ACCEPTED_4);
    
    // Set floor indicators (IND-003, IND-006)
    current_indicators &= ~(POS_FLOOR_2 | POS_FLOOR_3);
    current_indicators &= ~(CAB_POS_2 | CAB_POS_3);
    current_indicators |= POS_FLOOR_4 | CAB_POS_4;
    
    elevator_indicators(current_indicators);
}

void goingdnto2_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_DOWN); // MOV-002
    
    // Set occupied indicators and clear others (CALL-004)
    current_indicators |= UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4;
    
    // Set appropriate acceptance indicators (CALL-001, REQ-001)
    current_indicators |= REQ_FLOOR_ACCEPTED_2 | CALL_ACCEPTED_FLOOR_2;
    
    elevator_indicators(current_indicators);
}

void goingdnto3_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_DOWN); // MOV-002
    
    // Set occupied indicators and clear others (CALL-004)
    current_indicators |= UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4;
    
    // Set appropriate acceptance indicators (CALL-002, REQ-002)
    current_indicators |= REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3;
    
    elevator_indicators(current_indicators);
}

void goingupto3_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_UP); // MOV-001
    
    // Set occupied indicators and clear others (CALL-004)
    current_indicators |= UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4;
    
    // Set appropriate acceptance indicators (CALL-002, REQ-002)
    current_indicators |= REQ_FLOOR_ACCEPTED_3 | CALL_ACCEPTED_FLOOR_3;
    
    elevator_indicators(current_indicators);
}

void goingupto4_state_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(GO_UP); // MOV-001
    
    // Set occupied indicators and clear others (CALL-004)
    current_indicators |= UPPTAGEN_FLOOR_2 | UPPTAGEN_FLOOR_3 | UPPTAGEN_FLOOR_4;
    
    // Set appropriate acceptance indicators (CALL-003, REQ-003)
    current_indicators |= REQ_FLOOR_ACCEPTED_4 | CALL_ACCEPTED_FLOOR_4;
    
    elevator_indicators(current_indicators);
}

void floor2_door_opening_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(OPEN_DOOR); // DOOR-001
}

void floor2_door_closing_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(CLOSE_DOOR); // DOOR-002
}

void floor3_door_opening_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(OPEN_DOOR); // DOOR-001
}

void floor3_door_closing_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(CLOSE_DOOR); // DOOR-002
}

void floor4_door_opening_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(OPEN_DOOR); // DOOR-001
}

void floor4_door_closing_entry()
{
    DEBUG_PRINT("\n");
    elevator_control_cmd(CLOSE_DOOR); // DOOR-002
}

// Exit functions
void floor_state_exit()
{
    DEBUG_PRINT("\n");
    // Common exit actions for floor states
    elevator_control_cmd(CLOSE_DOOR); // Ensure door is closing when leaving floor state
}

void moving_state_exit()
{
    DEBUG_PRINT("\n");
    // Common exit actions for movement states
    elevator_control_cmd(STOP); // MOV-003 - Stop when exiting movement state
}

const char *elevatorStateEnumNames(elevatorStateEnum e)
{
    assert(e >= OFF && e < NUM_STATES);
    const char *n[] = {
        "OFF",
        "INIT",
        "FLOOR2",
        "FLOOR3",
        "FLOOR4",
        "GOINGUPTO3",
        "GOINGDNTO3",
        "GOINGUPTO4",
        "GOINGDNTO2",
        "FLOOR2_DOOR_OPENING",
        "FLOOR2_DOOR_CLOSING",
        "FLOOR3_DOOR_OPENING",
        "FLOOR3_DOOR_CLOSING",
        "FLOOR4_DOOR_OPENING",
        "FLOOR4_DOOR_CLOSING"
    };
    return n[e];
}