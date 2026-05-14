#ifndef OBJECTIVES_LOADER_H
#define OBJECTIVES_LOADER_H

#include <stdbool.h>
#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_objectives_from_file(const char* filename);

int get_objective_count(void);

#ifdef __cplusplus
}
#endif

#endif

