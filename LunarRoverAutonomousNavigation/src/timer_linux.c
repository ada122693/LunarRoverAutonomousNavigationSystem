#define _GNU_SOURCE

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>//
#include <sys/timerfd.h>//
#include <time.h>
#include <poll.h>//
#include <unistd.h>//
#include <string.h>

#include "constants/constants.h"
#include "constants/config.h"

#include "dispatcher.h"
#include "platform.h"//

struct platform_timer {
  int fd;
  uint64_t interval_ns;
};

//config for the timer
void setup_timer(struct itimerspec* timer, uint64_t interval_ns) {
  //convert to sec nsec format
  timer->it_interval.tv_sec = (interval_ns / NANOS_PER_SEC);
  timer->it_interval.tv_nsec = (interval_ns % NANOS_PER_SEC);

  timer->it_value = timer->it_interval;
}

platform_timer_t* platform_timer_create(uint64_t interval_ns) {
  // currently Linux Timer via timerfd
  int timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
  if (timer_fd == -1) {
    perror("Failed to create timerfd");
    return NULL;
  }

  struct itimerspec timer;
  memset(&timer, 0, sizeof(timer));

  /**timer config*/
  setup_timer(&timer, interval_ns);

  fprintf(stderr, "[TIMER] Setting timer interval to %lu ns (%.2f s)\n", interval_ns, interval_ns / 1e9);
  fprintf(stderr, "[TIMER] it_interval: %ld s %ld ns\n", timer.it_interval.tv_sec, timer.it_interval.tv_nsec);
  fprintf(stderr, "[TIMER] it_value: %ld s %ld ns\n", timer.it_value.tv_sec, timer.it_value.tv_nsec);

  if (timerfd_settime(timer_fd, 0, &timer, NULL) != 0) {
    perror("timerfd_settime failed");
    close(timer_fd);
    return NULL;
  }

  platform_timer_t* t = malloc(sizeof(platform_timer_t));
  if (!t) {
    close(timer_fd);
    return NULL;
  }

  t->fd = timer_fd;
  t->interval_ns = interval_ns;

  return t;
}

int platform_timer_wait(platform_timer_t* t, uint64_t* expirations) {
  struct pollfd pfd = { .fd = t->fd, .events = POLLIN };
  int ret = poll(&pfd, 1, -1);
  if (ret <= 0) return -1;

  return read(t->fd, expirations, sizeof(*expirations));
}

int platform_timer_get_fd(platform_timer_t* t) {
  if (!t) return -1;
  return t->fd;
}

void platform_timer_destroy(platform_timer_t* t) {
  if (!t) return;
  close(t->fd);
  free(t);
}