#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "structs.h"
#include "constants/constants.h"
#include "constants/config.h"
#include "constants/arrsize_global.h"

#include "deconf_pqueue.h"
#include "path_storage.h"
#include "rover_action_client.h"
#include "rover_config.h"
#include "gridmap/gridmap.h"
#include "objectives/objectives_loader.h"


// Global request queue for the system
static struct RequestQueue g_request_queue;
static bool g_system_initialized = false;

// Global variable for first slot wall time initialization
static double g_first_slot_wall_time = 0.0;
static int g_current_time_slot_offset = 0;

// Global reservation grid and intervals
static int g_sector_max = 0;
bool g_reservation_grid[SECTORS_ARR_SIZE][MAX_TIME_SLOTS_ARR_SIZE] = {0};
int g_intervals[SECTORS_ARR_SIZE][MAX_TIME_SLOTS_ARR_SIZE];

// Getter functions for reservation grid and intervals
int get_sector_max(void) {
    return g_sector_max;
}

bool* get_reservation_grid(void) {
    return *g_reservation_grid;
}

int* get_intervals(void) {
    return *g_intervals;
}

int get_time_slot_offset(void) {
    return g_current_time_slot_offset;
}

double get_first_slot_wall_time(void) {
    return g_first_slot_wall_time;
}

// Time shifting function for reservation grid and intervals
void shift_reservation_grid(int shift_amount) {
    if (shift_amount <= 0 || shift_amount >= MAX_TIME_SLOTS) {
        return;
    }
    
    // Shift each sector's reservation grid left
    for (int i = 0; i < g_sector_max; i++) {
        // memmove reservation grid left
        memmove(&g_reservation_grid[i][0], &g_reservation_grid[i][shift_amount], 
                (MAX_TIME_SLOTS - shift_amount) * sizeof(int));
        // memset the tail to 0
        memset(&g_reservation_grid[i][MAX_TIME_SLOTS - shift_amount], 0, 
               shift_amount * sizeof(int));
        
        // memmove intervals left
        memmove(&g_intervals[i][0], &g_intervals[i][shift_amount], 
                (MAX_TIME_SLOTS - shift_amount) * sizeof(int));
        // memset the tail to MAX_TIME_SLOTS
        memset(&g_intervals[i][MAX_TIME_SLOTS - shift_amount], MAX_TIME_SLOTS, 
               shift_amount * sizeof(int));
    }
    
    // Update time slot offset
    g_current_time_slot_offset += shift_amount;
    
    // Update first slot wall time to current time
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    g_first_slot_wall_time = ts.tv_sec + ts.tv_nsec / 1e9;
}

//for anything that needs initialization before the main system loop runs
int system_initialization() {
    fprintf(stderr, "[INIT] entering init\n");
    if (g_system_initialized) {
        return 0; // Already initialized
    }
    // Initialize grid map (must be done before pathfinding)
    fprintf(stderr, "[INIT] constructing map\n");
    init_gridmap();
    fprintf(stderr, "collision at (2,2): %d\n", check_collision(2.0, 2.0));
fprintf(stderr, "collision at (2.5,2): %d\n", check_collision(2.5, 2.0));
fprintf(stderr, "collision at (3,2): %d\n", check_collision(3.0, 2.0));
    fprintf(stderr, "[INIT] map constructed, calculating sectors\n");

	//reservation grid, must be initialized at the beginning before entering the system loop
    g_sector_max = (X_DIM_SIZE / SECTOR_WIDTH) * (Y_DIM_SIZE / SECTOR_HEIGHT);
    
    fprintf(stderr, "[INIT] sectors calculated, setting up clock stuff\n");
    // Initialize first slot wall time
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    g_first_slot_wall_time = ts.tv_sec + ts.tv_nsec / 1e9;
    
    fprintf(stderr, "[INIT] clock stuff set up, initializing intervals\n");
    // Initialize intervals to MAX_TIME_SLOTS (sentinel value)
    for (int i = 0; i < g_sector_max; i++) {
        for (int j = 0; j < MAX_TIME_SLOTS; j++) {
            g_intervals[i][j] = MAX_TIME_SLOTS;
        }
    }

    fprintf(stderr, "[INIT] intervals initialized, initializing rover action clients\n");
    // Initialize rover action clients from config file
    if (init_rover_action_clients(ROVERS_FILENAME) != 0) {
        fprintf(stderr, "Failed to initialize rover action clients\n");
        return -1;
    }

    fprintf(stderr, "[INIT] rover action clients initialized, initializing request queue\n");
    // Initialize request queue
    init_request_queue(&g_request_queue, 100); // Capacity of 100 requests
    
    fprintf(stderr, "[INIT] request queue initialized, getting actual number of rovers\n");
    // Get actual number of rovers from config file
    int actual_num_rovers = get_actual_num_rovers();
    if (actual_num_rovers == 0) {
        fprintf(stderr, "Warning: No rovers found in config file, using default MAX_ROVERS\n");
        actual_num_rovers = MAX_ROVERS; // Fallback to MAX_ROVERS
    }
    
    fprintf(stderr, "[INIT] actual number of rovers determined, initializing deconfliction priority queue\n");
    // Initialize deconfliction priority queue
    // This is the shared pending queue used by both instructions and deconf
    // Size is based on actual number of rovers
    deconf_pqueue_init(actual_num_rovers);
    
    fprintf(stderr, "[INIT] deconfliction priority queue initialized, loading existing paths from storage\n");
    // Load existing paths from storage
    load_instruction_package_from_file(PATH_FILENAME);
    
    fprintf(stderr, "[INIT] existing paths loaded, initializing objectives from file\n");
    // Initialize objectives from file
    init_objectives_from_file(OBJECTIVES_FILENAME);
    
    

    g_system_initialized = true;
    fprintf(stderr, "[INIT] system initialized successfully\n");
    return 0;
}

// Get the global request queue
struct RequestQueue* get_request_queue(void) {
    if (!g_system_initialized) {
        return NULL;
    }
    return &g_request_queue;
}

// Cleanup function for system shutdown
void system_cleanup(void) {
    if (!g_system_initialized) {
        return;
    }
    
    shutdown_rover_action_clients();
    free_request_queue(&g_request_queue);
    deconf_pqueue_cleanup();
    cleanup_path_storage();
    g_system_initialized = false;
}
