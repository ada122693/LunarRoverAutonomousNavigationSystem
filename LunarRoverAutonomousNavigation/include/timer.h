#ifndef TIMER_H
#define TIMER_H

#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Signal handler for shutdown */
void shutdown_handler(int sig);

/** config for the timer */
void setup_timer(int timer_fdu);

#ifdef __cplusplus
}
#endif

#endif
