# elevatorSimulator


This program provides an example of an elevator simulator.  It was created
for educational purposes (specifically CSE3310).

Think of this system as 3 parts:

1. The elevator GUI, which provides a way to provide stimulus (events) to the system. It is coded in the C++ language.  A majority was created using the fltk gui layout program known as 'fluid'.
2. The elevator, which simulates the physical elevator. This is coded in the C programming language.
3. The elevator controller, which controls the physical elevator based upon the inputs recieved.  It is coded in the C language.

to change the UI:

```
cd ./src
fluid elevatorGUI.fl
```

to compile the source code

```
make clean
make
```

to execute the program

```
./eSim
```


Requirements:
(Note, these requirements do not cover the Elevator Controller, only the elevator simulator)

For the Elevator simulator.

| REQ ID | Text                                                     |Implemented           | Tested |
|:-------| :---                                                     |:---                  |:---    |
| 001    | The elevator door shall open when a DOOR_OPEN command is | elevator_tick()      | TC002  |
|        | received.                                                |                      |        |
| 003    | The elevator door shall close when a DOOR_CLOSE command  | elevator_tick()      | TC003  |
|        | is received.                                             |                      |        |
| 002    | The elevator door will open or close in 5 seconds,       | elevator_tick()      | TC003  |
|        | +- 1 second                                              |                      |        |
| 004    | The elevator shall STOP when an elevator command of stop | elevator_tick()      |        |
|        | is recieved.                                             |                      |        |
| 005    | The elevator shall descend when an elevator command of   | elevator_tick()      | TC003  |
|        | GO_DOWN is recieved.                                     |                      |        |
| 006    | The elevator shall ascend when an elevator command of    | elevator_tick()      | TC003  |
|        | GO_UP is recieved.                                       |                      |        |
| 007    | The elevator shall move up or down at the rate of 5      | elevator_tick()      | TC003  |
|        | seconds per floor, +- 2 seconds per floor.               |                      |        |
| 008    | When power is turned off, the elevator does nothing.     | elevator_tick()      |        |
| 009    | When power is turned on, the elevator is initialized     | init_elevator()      |        |
|        | as follows:                                              |                      |        |
|        |        power = ELEV_POWER_OFF;                           |                      |        |
|        |        cabPosition = ELEV_CAB_POSITION_2;                |                      |        |
|        |        cabDirection = ELEV_CAB_DIRECTION_NEUTRAL;        |                      |        |
|        |        doorPosition = ELEV_DOOR_POSITION_CLOSED;         |                      |        |
|        |        doorDirection = LEV_DOOR_DIRECTION_NOT_MOVING;    |                      |        |
| 020    | The elevator shall create an event when it arrives at    | elevator_tick()      |        |
|        | floor2.                                                  |                      |        |
| 021    | The elevator shall create an event when it arrives at    | elevator_tick()      |        |
|        | floor 2.5                                                |                      |        |
| 022    | The elevator shall create an event when it arrives at    | elevator_tick()      |        |
|        | floor 3                                                  |                      |        |
| 023    | The elevator shall create an event when it arrives at    | elevator_tick()      |        |
|        | floor 3.5                                                |                      |        |
| 024    | The elevator shall create an event when it arrives at    | elevator_tick()      |        |
|        | floor 4                                                  |                      |        |
| 011    | The elevator will ignore invalid control combinations:   | elevator_control_cmd |TC001   |
|        |   GO_UP & GO_DOWN                                        |                      |        |
|        |   GO_UP & GO_DOWN & STOP                                 |                      |        |
|        |   GO_UP &  STOP                                          |                      |        |
|        |   STOP & GO_DOWN                                         |                      |        |
 