
 //these are for testing only and do not define actual behavior that should be
 //performed if a rover dies or needs to be killed

void dead_rover(struct Rover *rover) {
}

/*
void rover_death_in_flight(struct Rover *rover) {
}*/

void set_rover_kill(struct Rover *rover) {
    //assign instructions to wait indefinitely at current position
}

void send_rover_idle(struct Rover *rover) {
    //platform-specific: send idle command to rover
    //for a preset period of time, NOT INDEFINITE
    //cause of how the shims work, that will kill it
}

void set_rover_idle(struct Rover *rover) {
    //mark rover as idle in system
}
