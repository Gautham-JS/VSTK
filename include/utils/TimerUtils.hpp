#include <stdio.h>
#include <string>
#include <chrono>

#ifndef __VSTK_CHRONO_UTIL_H
#define __VSTK_CHRONO_UTIL_H

namespace vstk {
    typedef std::chrono::_V2::system_clock::time_point time_pt;
    typedef std::chrono::milliseconds ms_unit_t;
    typedef std::chrono::nanoseconds ns_unit_t;


    typedef struct TimerContainer {
        std::string marker;
        time_pt start_t;
        time_pt end_t;

        uint64_t elapsed_ms;
        uint64_t elapsed_ns;
        
    } Timer;


    void start_timer(vstk::Timer &timer);

    void end_timer(vstk::Timer &timer);

    vstk::Timer get_timer(std::string marker);

    void log_timer(vstk::Timer timer, FILE *fptr);
    void log_fps(vstk::Timer timer, FILE* fptr); 
    double get_fps(vstk::Timer timer);
}

#endif
