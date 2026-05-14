#ifndef DEAD_ROVER_H
#define DEAD_ROVER_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

//these are for testing only and do not define actual behavior that should be
//performed if a rover dies or needs to be killed

void dead_rover(struct Rover *rover);
void set_rover_kill(struct Rover *rover);
void send_rover_idle(struct Rover *rover);
void set_rover_idle(struct Rover *rover);
// void rover_death_in_flight(struct Rover *rover); // Commented out

#ifdef __cplusplus
}
#endif

#endif
