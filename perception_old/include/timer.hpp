#ifndef TIMER_H
#define TIMER_H

#ifdef __GNUC__
#else
#include <windows.h>
#endif

#include "basic_struct.h"

#ifdef __GNUC__
class Timer {
public:
    int do_start_timer() {
        clock_gettime(CLOCK_REALTIME, &_time1);
        return 0;
    }

    int do_end_timer(std::string prefix) {
        clock_gettime(CLOCK_REALTIME, &_time2);
        std::cout << prefix << (_time2.tv_sec - _time1.tv_sec) * 1000.0 +
                    (_time2.tv_nsec - _time1.tv_nsec) /
                    1000000.0 << "ms ####" << std::endl;
        return 0;
    }

private:
    struct timespec _time1;
    struct timespec _time2;

};

#else

class Timer {
public:
    Timer() {
        QueryPerformanceFrequency(&_tc);
    }

    int do_start_timer() {
        QueryPerformanceCounter(&_t1);
        return 0;
    }

    int do_end_timer(std::string prefix) {
        QueryPerformanceCounter(&_t2);
        std::cout << prefix << (_t2.QuadPart - _t1.QuadPart) * 1000.0 /
            _tc.QuadPart << "ms ####" << std::endl;
        return 0;
    }

private:
    LARGE_INTEGER _t1;
    LARGE_INTEGER _t2;
    LARGE_INTEGER _tc;

};

#endif

#endif