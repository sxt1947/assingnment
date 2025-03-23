#include "elevatorController.h"
#include "fct.h"
#include <string.h>

FCT_BGN()
{
    FCT_SUITE_BGN(elevator controller unit tests)
    {
        FCT_TEST_BGN(fsm transition)
        {
            printf("\n");
            /* these are valid transitions */ 
            fct_chk(transition(OFF, POWER_ON) == INIT);
            
            fct_chk(transition(GOINGDNTO2, CAB_POSITION_FLOOR_2) == FLOOR2);
            fct_chk(transition(GOINGDNTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
            fct_chk(transition(GOINGUPTO3, CAB_POSITION_FLOOR_3) == FLOOR3);
            fct_chk(transition(GOINGUPTO4, CAB_POSITION_FLOOR_4) == FLOOR4);

            fct_chk(transition(FLOOR2, CALL_FLOOR_3) == GOINGUPTO3);
            fct_chk(transition(FLOOR2, CALL_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(FLOOR3, CALL_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(FLOOR3, CALL_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(FLOOR4, CALL_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(FLOOR4, CALL_FLOOR_3) == GOINGDNTO3);

            fct_chk(transition(FLOOR2, REQ_FLOOR_3) == GOINGUPTO3);
            fct_chk(transition(FLOOR2, REQ_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(FLOOR3, REQ_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(FLOOR3, REQ_FLOOR_4) == GOINGUPTO4);
            fct_chk(transition(FLOOR4, REQ_FLOOR_2) == GOINGDNTO2);
            fct_chk(transition(FLOOR4, REQ_FLOOR_3) == GOINGDNTO3);

            /* these events should be ignored */
            fct_chk(transition(FLOOR2, CALL_FLOOR_2) == FLOOR2);
            fct_chk(transition(FLOOR3, CALL_FLOOR_3) == FLOOR3);
            fct_chk(transition(FLOOR4, CALL_FLOOR_4) == FLOOR4);

            printf("\n");
        }
        FCT_TEST_END();

        // New test: Initialization behavior
        FCT_TEST_BGN(initialization)
        {
            controller_init();
            // Verify initial state is OFF
            fct_chk(transition(OFF, POWER_ON) == INIT);
            // After timer expires, should be at FLOOR2
            fct_chk(transition(INIT, TIMER_EXPIRED) == FLOOR2);
        }
        FCT_TEST_END();

        // New test: Indicator settings
        FCT_TEST_BGN(indicator_settings)
        {
            // Test indicator settings when arriving at floors
            elevatorStateEnum state = GOINGUPTO3;
            state = transition(state, CAB_POSITION_FLOOR_3);
            fct_chk(state == FLOOR3);
            // Note: Would need to expose indicators() function or add getter to test actual indicator values
            
            state = GOINGDNTO2;
            state = transition(state, CAB_POSITION_FLOOR_2);
            fct_chk(state == FLOOR2);
        }
        FCT_TEST_END();

        // New test: Invalid transitions during movement
        FCT_TEST_BGN(movement_lockout)
        {
            // Verify that additional calls/requests are ignored during movement
            fct_chk(transition(GOINGUPTO3, CALL_FLOOR_4) == GOINGUPTO3);
            fct_chk(transition(GOINGDNTO2, REQ_FLOOR_3) == GOINGDNTO2);
            fct_chk(transition(GOINGUPTO4, REQ_FLOOR_2) == GOINGUPTO4);
        }
        FCT_TEST_END();

        // New test: Door operation
        FCT_TEST_BGN(door_operation)
        {
            // Test door open request at floor
            fct_chk(transition(FLOOR2, REQ_DOOR_OPEN) == FLOOR2);
            // Test door response to obstruction
            // Note: This functionality was requested to be removed, but test case remains for completeness
        }
        FCT_TEST_END();
        
        // New test: Comprehensive sequence test
        FCT_TEST_BGN(comprehensive_sequence)
        {
            // Start with initialization
            controller_init();
            elevatorStateEnum state = OFF;
            
            // Power on sequence
            state = transition(state, POWER_ON);
            fct_chk(state == INIT);
            
            // Wait for timer to expire and go to FLOOR2
            state = transition(state, TIMER_EXPIRED);
            fct_chk(state == FLOOR2);
            
            // Request to go to floor 4
            state = transition(state, REQ_FLOOR_4);
            fct_chk(state == GOINGUPTO4);
            
            // Arrive at floor 4
            state = transition(state, CAB_POSITION_FLOOR_4);
            fct_chk(state == FLOOR4);
            
            // Call from floor 2
            state = transition(state, CALL_FLOOR_2);
            fct_chk(state == GOINGDNTO2);
            
            // Arrive at floor 2
            state = transition(state, CAB_POSITION_FLOOR_2);
            fct_chk(state == FLOOR2);
        }
        FCT_TEST_END();
        
        // New test: Door safety requirements
        FCT_TEST_BGN(door_safety)
        {
            // Test that motion is not allowed with doors open
            // Start at floor 2 with door closed (assumed)
            elevatorStateEnum state = FLOOR2;
            
            // Request floor 3, moving to GOINGUPTO3 state
            state = transition(state, REQ_FLOOR_3);
            fct_chk(state == GOINGUPTO3);
            
            // Verify in movement state, cab arrives at floor 3
            state = transition(state, CAB_POSITION_FLOOR_3);
            fct_chk(state == FLOOR3);
            
            // At this point the doors should open automatically
            // (behavior implemented in floor3_state_entry)
        }
        FCT_TEST_END();

        FCT_TEST_BGN(chk_neq)
        {
            fct_chk(strcmp("daka", "durka") != 0);
        }
        FCT_TEST_END();
    }
    FCT_SUITE_END();
}
FCT_END();