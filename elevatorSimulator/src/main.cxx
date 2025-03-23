#include <string>

#include <FL/Fl_Timer.H>

#include "elevatorGUI.h"
#include "elevator.h"
#include "elevatorController.h"


static UserInterfaceElevator *window = nullptr;

extern "C" void callback(void*) 
{       
        // need to update each part of the system
        // as time goes on...
        elevator_tick();
        controller_tick();
        if (window)
        {
                window->guiTick();
        }

        // call me back in 1 second, please
        Fl::repeat_timeout(1.0, callback);
}
 
int main(int argc, char *argv[]) 
{
        window = new UserInterfaceElevator();
        Fl_Window *w = window->make_window();
        w->show(argc, argv);

        Fl::add_timeout(1.0, callback);

        return Fl::run();
}
