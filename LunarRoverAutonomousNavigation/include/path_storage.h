#ifndef PATH_STORAGE_H
#define PATH_STORAGE_H

#include "structs.h"

#ifdef __cplusplus
extern "C" {
#endif

// File I/O functions for instruction packages
int save_instruction_package_to_file(struct InstructionPackage* package, const char* filename);
struct InstructionPackage* load_instruction_package_from_file(const char* filename);

// Path matching function
struct InstructionPackage* find_matching_path(const struct Location* start, const struct Location* end);

// Save current path during generation
int save_current_path(struct InstructionPackage* package, const struct Location* start, const struct Location* end);

// Cleanup function
void cleanup_path_storage(void);

#ifdef __cplusplus
}
#endif

#endif
