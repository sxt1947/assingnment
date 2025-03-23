#include "elevatorController.h"

// https://github.com/imb/fctx/tree/master

#include "fct.h"
#include <string.h>

FCT_BGN()
{
    
   FCT_SUITE_BGN(elevator controller unit tests)
   {

      FCT_TEST_BGN(fsm transition)
      {
         printf("\n");
         /*
            first, check the most basic thing.  is the fsm table done correctly?
            lets pick a few transitions to test.

         

            these tests will break if the table changes....not ideal, but the way it is.
         */
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

      FCT_TEST_BGN(chk_neq)
      {
         fct_chk(strcmp("daka", "durka") != 0);
      }
      FCT_TEST_END();

      /* Every test suite must be closed. */
   }
   FCT_SUITE_END();

   /* Every FCT scope has an end. */
}
FCT_END();
