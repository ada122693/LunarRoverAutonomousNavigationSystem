#ifndef PLATFORM_H
#define PLATFORM_H

//detects platforms and does stuff so I can run it in windows or linux
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Platform detection
#if defined(_WIN32) || defined(_WIN64)
    #define PLATFORM_WINDOWS
#elif defined(__linux__)
    #define PLATFORM_LINUX
#elif defined(__APPLE__)
    #define PLATFORM_MACOS
#else
    #error "Unsupported platform"
#endif

// Platform-specific includes
#ifdef PLATFORM_WINDOWS
    #include <windows.h>
#elif defined(PLATFORM_LINUX)
    #include <unistd.h>
    #include <sys/timerfd.h>
    #include <poll.h>
#endif

typedef struct platform_timer platform_timer_t;

// Platform timer functions
platform_timer_t* platform_timer_create(uint64_t interval_ns);
int platform_timer_get_fd(platform_timer_t* timer);
void platform_timer_destroy(platform_timer_t* timer);

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_H