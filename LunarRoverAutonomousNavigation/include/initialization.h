#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// System initialization and cleanup
int system_initialization(void);
void system_cleanup(void);

// Time synchronization
void sync_wall_time(void);

// Reservation grid management
void shift_reservation_grid(int shift_amount);

#ifdef __cplusplus
}
#endif

#endif
