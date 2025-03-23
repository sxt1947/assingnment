#include "elevatorController.h"
#include "fct.h"
#include <string.h>

FCT_BGN()
{
    FCT_SUITE_BGN(elevator controller unit tests)
    {
        FCT_TEST_BGN(initialization)
        {
            controller_init();
            fct_chk(get_current_state() == OFF);
            event_to_controller(POWER_ON);
            fct_chk(get_current_state() == INIT);
            fct_chk(get_timer() == 20);
        }
        FCT_TEST_END();
        
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

        // Add more tests as needed
    }
    FCT_SUITE_END();
}
FCT_END();