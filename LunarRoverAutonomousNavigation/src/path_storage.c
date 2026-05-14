#include "structs.h"
#include "constants/constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "config.h"


//for loaded paths
static struct InstructionPackage* g_stored_paths = NULL;
static int g_stored_paths_count = 0;
static int g_stored_paths_capacity = 0;

static double location_distance(const struct Location* a, const struct Location* b) {
    double dx = a->x_dim - b->x_dim;
    double dy = a->y_dim - b->y_dim;
    return sqrt(dx * dx + dy * dy);
}

// approximately equal (within tolerance)
static bool locations_equal(const struct Location* a, const struct Location* b, double tolerance) {
    return fabs(a->x_dim - b->x_dim) < tolerance &&
           fabs(a->y_dim - b->y_dim) < tolerance;
}

int save_instruction_package_to_file(struct InstructionPackage* package, const char* filename) {
    if (!package || !filename) return -1;

    // tmp
    struct InstructionPackage storage_package = *package;
    storage_package.metadata.rover_id = -1; // dummy ID
    storage_package.metadata.priority = -1; // dummy priority = nothing
    storage_package.metadata.num_waits = 0;

    FILE* file = fopen(filename, "ab");
    if (!file) return -1;

    size_t written = fwrite(&storage_package, sizeof(struct InstructionPackage), 1, file);
    fclose(file);

    return (written == 1) ? 0 : -1;
}

struct InstructionPackage* load_instruction_package_from_file(const char* filename) {
    if (!filename) return NULL;

    FILE* file = fopen(filename, "rb");
    if (!file) return NULL;

    if (g_stored_paths) {
        free(g_stored_paths);
        g_stored_paths = NULL;
    }
    g_stored_paths_count = 0;
    g_stored_paths_capacity = 0;

    // all from file
    struct InstructionPackage package;
    const int MAX_PATHS_TO_LOAD = 100;
    while (fread(&package, sizeof(struct InstructionPackage), 1, file) == 1 && g_stored_paths_count < MAX_PATHS_TO_LOAD) {
        // expand
        if (g_stored_paths_count >= g_stored_paths_capacity) {
            int new_capacity = (g_stored_paths_capacity == 0) ? 16 : g_stored_paths_capacity * 2;
            if (new_capacity > MAX_PATHS_TO_LOAD) {
                new_capacity = MAX_PATHS_TO_LOAD;
            }
            struct InstructionPackage* new_array = realloc(g_stored_paths, new_capacity * sizeof(struct InstructionPackage));
            if (!new_array) {
                fclose(file);
                return NULL;
            }
            g_stored_paths = new_array;
            g_stored_paths_capacity = new_capacity;
        }
        g_stored_paths[g_stored_paths_count++] = package;
    }
    fclose(file);

    return (g_stored_paths_count > 0) ? g_stored_paths : NULL;
}

struct InstructionPackage* find_matching_path(const struct Location* start, const struct Location* end) {
    if (g_stored_paths_count == 0) {
        return NULL; // None
    }

    for (int i = 0; i < g_stored_paths_count; i++) {
        struct InstructionPackage* candidate = &g_stored_paths[i];
        if (!candidate->metadata.valid) continue;

        // approximate
        if (locations_equal(&candidate->metadata.start_location, start, LOCATION_EPSILON) &&
            locations_equal(&candidate->metadata.end_location, end, LOCATION_EPSILON)) {
            return candidate;
        }
    }

    return NULL;
}

// saving paths/instructions to files
int save_current_path(struct InstructionPackage* package, const struct Location* start, const struct Location* end) {
    if (!package) return -1;
    package->metadata.start_location = *start;
    package->metadata.end_location = *end;

   return save_instruction_package_to_file(package, PATH_FILENAME);
}

void cleanup_path_storage(void) {
    if (g_stored_paths) {
        free(g_stored_paths);
        g_stored_paths = NULL;
    }
    g_stored_paths_count = 0;
    g_stored_paths_capacity = 0;
}
