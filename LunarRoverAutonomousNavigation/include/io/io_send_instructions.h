#ifndef IO_SEND_INSTRUCTIONS_H
#define IO_SEND_INSTRUCTIONS_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

//sends with ros2 action client
int send_instructions(struct PlatformSpecificInstructions* instructions, double actual_dispatch_time);

#ifdef __cplusplus
}
#endif

#endif

