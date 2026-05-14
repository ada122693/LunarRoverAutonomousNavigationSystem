//this is a timer thing again but for windows
//I made it cause I was writing code on a windows computer as well as linux
//WILL NOT WORK anymore cause the program is entirely ros2 dependent now
//left here for future development
#include "platform.h"
#include <windows.h>
#include <stdlib.h>
#include "constants/config.h"

struct platform_timer {
    HANDLE timer;
    LARGE_INTEGER interval;
};

platform_timer_t* platform_timer_create() {
    platform_timer_t* t = malloc(sizeof(platform_timer_t));
    if (!t) return NULL;
    
    t->timer = CreateWaitableTimer(NULL, FALSE, NULL);
    if (!t->timer) {
        free(t);
        return NULL;
    }

    //already in ns :3
    t->interval.QuadPart = -(LONGLONG)PROACTIVE_MONITORING_INTERVAL;
    SetWaitableTimer(t->timer, &t->interval, (LONG)PROACTIVE_MONITORING_INTERVAL, NULL, NULL, FALSE);
    
    return t;
}

int platform_timer_wait(platform_timer_t* t, uint64_t* expirations) {
    DWORD result = WaitForSingleObject(t->timer, INFINITE);
    if (result != WAIT_OBJECT_0) {
        return -1;
    }

    *expirations = 1;
    return 0;
}

void platform_timer_destroy(platform_timer_t* t) {
    if (!t) return;
    CloseHandle(t->timer);
    free(t);
}
