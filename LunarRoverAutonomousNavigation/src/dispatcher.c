#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>

#include "rover_action_client.h"

#include "structs.h"
#include "constants/constants.h"
#include "constants/config.h"

#include "queue_management.h"
#include "requests/requests_process.h"
#include "send/send_main.h"
#include "initialization.h"
#include "dispatcher.h"
#include "platform.h"


// External function declaration for shift_reservation_grid
extern void shift_reservation_grid(int shift_amount);

// External function declaration for get_first_slot_wall_time
extern double get_first_slot_wall_time(void);

// Time-based tracking for periodic operations (deadline-based)
static struct timespec next_sync_time;
static struct timespec next_schedule_time;
static struct timespec next_shift_time;

// ensure constants don't violate logic
static bool is_configuration_valid(void) {
  if (PROACTIVE_MONITORING_INTERVAL <= 0 || SCHEDULING_PACE <= 0) {
    return false;
  }
  return true;
}

// Helper function to add nanoseconds to a timespec
static void add_ns_to_timespec(struct timespec *ts, uint64_t ns) {
  ts->tv_sec += ns / 1000000000ULL;
  ts->tv_nsec += ns % 1000000000ULL;
  if (ts->tv_nsec >= 1000000000L) {
    ts->tv_sec++;
    ts->tv_nsec -= 1000000000L;
  }
}

// Helper function to compare timespecs (returns true if a >= b)
static bool timespec_ge(struct timespec *a, struct timespec *b) {
  if (a->tv_sec > b->tv_sec) return true;
  if (a->tv_sec == b->tv_sec && a->tv_nsec >= b->tv_nsec) return true;
  return false;
}

// for handling the actual timing in the system
bool handle_timer_events(uint64_t missed) {
  // halt as missing too many ticks indicates system overload
  if (missed > MAX_ALLOWED_JITTER_TICKS) {
    fprintf(stderr, "CRITICAL: Timing overrun detected. Halting for safety.\n");
    return false;
  }

  // Snapshot current time once
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  // Process any pending requests
  process_requests();

  // deconf + sending schedules and paths
  if (timespec_ge(&now, &next_schedule_time)) {
    create_and_send_rover_schedules();
    next_schedule_time = now;
    uint64_t schedule_interval_ns = SCHEDULING_PACE * PROACTIVE_MONITORING_INTERVAL;
    add_ns_to_timespec(&next_schedule_time, schedule_interval_ns);
  }

  // shift reservation grid - prevents overflow
  if (timespec_ge(&now, &next_shift_time)) {
    fprintf(stderr, "[STATUS] Updating schedule...\n");
    shift_reservation_grid(MAX_TIME_SLOTS / 4);
    fprintf(stderr, "[STATUS] Schedule updated.\n");
    next_shift_time = now;
    uint64_t shift_threshold_ns = (uint64_t)((MAX_TIME_SLOTS * TIME_SLOT_SIZE * 1e9) / 4.0);
    add_ns_to_timespec(&next_shift_time, shift_threshold_ns);
  }

  // syncing /clock to force the robots to sync
  if (timespec_ge(&now, &next_sync_time)) {
    fprintf(stderr, "[STATUS] Telling the robots to check the time...\n");
    sync_wall_time();
    next_sync_time = now;
    add_ns_to_timespec(&next_sync_time, (uint64_t)(TIME_BEFORE_SYNC_MS * 1000000.0));
    fprintf(stderr, "[STATUS] Time synchronized\n");
  }

  return true;
}


void run_system_loop(platform_timer_t* timer, volatile bool *keep_running) {
  fprintf(stderr, "[STATUS] Starting system loop\n");
  if (!is_configuration_valid()) {
    fprintf(stderr, "CRITICAL: Invalid timing constants. System halted.\n");
    return;
  }

  // deadlines
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  
  next_sync_time = now;
  add_ns_to_timespec(&next_sync_time, (uint64_t)(TIME_BEFORE_SYNC_MS * 1000000.0));
  
  next_schedule_time = now;
  uint64_t schedule_interval_ns = SCHEDULING_PACE * PROACTIVE_MONITORING_INTERVAL;
  add_ns_to_timespec(&next_schedule_time, schedule_interval_ns);
  
  next_shift_time = now;
  uint64_t shift_threshold_ns = (uint64_t)((MAX_TIME_SLOTS * TIME_SLOT_SIZE * 1e9) / 4.0);
  add_ns_to_timespec(&next_shift_time, shift_threshold_ns);

  int timer_fd = platform_timer_get_fd(timer);
  struct pollfd fds = {.fd = timer_fd, .events = POLLIN};
  uint64_t missed;

  fprintf(stderr, "[STATUS] Entering main loop\n");
  while (*keep_running) {
    int ready = poll(&fds, 1, -1);

    if (ready == 0) {
      fprintf(stderr, "CRITICAL: Unexpected poll() timeout.\n");
      break;
    }

    if (ready < 0) {
      fprintf(stderr, "CRITICAL: Poll error.\n");
      break;
    }

    ssize_t n = read(timer_fd, &missed, sizeof(missed));

    if (n != sizeof(missed)) {
      fprintf(stderr, "CRITICAL: timerfd read had unexpected size.\n");
      break;
    }

    handle_timer_events(missed);

    if (missed > 1) {
      fprintf(stderr, "WARN: timer lag detected = %lu\n", missed);
    }
  }
  fprintf(stderr, "[STATUS] System exited, have a nice day :)\n");
}
