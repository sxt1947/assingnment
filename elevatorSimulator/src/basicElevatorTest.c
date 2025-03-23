#include "events.h"
#include "elevator.h"

// https://github.com/imb/fctx/tree/master

#include "fct.h"
#include <string.h>

// These functions are used for the tests in this file
// only.
static eventEnum last_event = -1;
void event_to_controller(eventEnum e)
{
   last_event = e;
}

static int init_called = 0;
void controller_init()
{
   init_called = 1;
}

FCT_BGN()
{
   FCT_SUITE_BGN(misc things)
   {
      FCT_TEST_BGN(TC001)
      {
         printf("\n");
         /*
            detect and ignore commands that do
            not make sense.
         */
         fct_chk(elevator_control_cmd(GO_UP | GO_DOWN) == -1);
         fct_chk(elevator_control_cmd(GO_UP | GO_DOWN | STOP) == -1);
         fct_chk(elevator_control_cmd(GO_UP | STOP) == -1);
         fct_chk(elevator_control_cmd(STOP | GO_DOWN) == -1);

         printf("\n");
      }
      FCT_TEST_END();
   }
   FCT_SUITE_END();
   FCT_SUITE_BGN(door)
   {
      FCT_TEST_BGN(TC002)
      {
         printf("\n");

         // initialize the elevator
         init_elevator();

         // turn it on
         power_on();

         // the door should be closed
         fct_chk(door_position() == 5); // 5 is closed */

         // send the command to open the door
         elevator_control_cmd(OPEN_DOOR);
         // tick() it
         for (int i = 0; i < 5; i++) // 5 seconds
         {
            elevator_tick();
            printf("the door pos is %d\n", door_position());
         }
         // test if it is open
         fct_chk(door_position() == 0);
         printf("\n");
      }
      FCT_TEST_END();
      FCT_TEST_BGN(TC003)
      {
         printf("\n");

         // initialize the elevator
         init_elevator();

         // turn it on
         power_on();

         // the door should be closed
         fct_chk(door_position() == 5); // 5 is closed */

         // send the command to open the door
         elevator_control_cmd(OPEN_DOOR);

         // tick() it
         for (int i = 0; i < 5; i++) // 5 seconds
         {
            elevator_tick();
         }
         // test if it is open
         fct_chk(door_position() == 0);

         // Now close it

         // send the command to close the door
         elevator_control_cmd(CLOSE_DOOR);

         // tick() it
         for (int i = 0; i < 5; i++) // 5 seconds
         {
            elevator_tick();
         }
         // test if it is closed
         fct_chk(door_position() == 5);
         printf("\n");
      }
      FCT_TEST_END();
   }
   FCT_SUITE_END();
   FCT_SUITE_BGN(cab movement)
   {
      FCT_TEST_BGN(up)
      {
         printf("\n");

         // initialize the elevator
         init_elevator();

         // turn it on
         power_on();
         // should be on the second floor

         fct_chk(cab_position() == ELEV_CAB_POSITION_2);

         // send it up
         elevator_control_cmd(GO_UP);

         // tick() it
         for (int i = 0; i < 10; i++) // 10 seconds
         {
            elevator_tick();
         }
         // test if it is at floor 3
         fct_chk(cab_position() == ELEV_CAB_POSITION_3);

         // go for 10 more seconds

         // send it up
         elevator_control_cmd(GO_UP);

         // tick() it
         for (int i = 0; i < 10; i++) // 10 seconds
         {
            elevator_tick();
         }

         // test if it is at floor 4
         fct_chk(cab_position() == ELEV_CAB_POSITION_4);

         printf("\n");
      }
      FCT_TEST_END();
   }
   FCT_SUITE_END();
   FCT_SUITE_BGN(ets)
   {
      FCT_TEST_BGN(u)
      {
      }
      FCT_TEST_END();
   }
   FCT_SUITE_END();

   FCT_END();
}

