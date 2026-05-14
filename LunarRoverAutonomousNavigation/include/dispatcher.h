//timer stuff

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
    #endif

    typedef struct platform_timer platform_timer_t;

    void run_system_loop(platform_timer_t* timer, volatile bool* keep_running);
    bool handle_timer_events(uint64_t missed);
    
    //runs the request handler, proavo, and nav
    int process_requests(void);

    #ifdef __cplusplus
}
#endif
