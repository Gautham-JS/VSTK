#include "utils/TimerUtils.hpp"
#include "utils/Logger.hpp"

void vstk::start_timer(vstk::Timer &timer) {
    timer.start_t = std::chrono::high_resolution_clock::now();
}

void vstk::end_timer(vstk::Timer &timer) {
    timer.end_t = std::chrono::high_resolution_clock::now();
    vstk::ms_unit_t ms = std::chrono::duration_cast<vstk::ms_unit_t>(timer.end_t - timer.start_t);
    vstk::ns_unit_t ns = std::chrono::duration_cast<vstk::ns_unit_t>(timer.end_t - timer.start_t);
    timer.elapsed_ms = ms.count();
    timer.elapsed_ns = ns.count();
}

vstk::Timer vstk::get_timer(std::string marker) {
    vstk::Timer timer;
    timer.marker = marker;
    return timer;
}

void vstk::log_timer(vstk::Timer timer, FILE* fptr) {
    fprintf(fptr, "\n======================================\nMarker : %s\nCompute Time : %ld ms\n======================================\n", timer.marker.c_str(), timer.elapsed_ms);
}

double vstk::get_fps(vstk::Timer timer) {
    return (1000 / ((double) timer.elapsed_ms));
}

void vstk::log_fps(vstk::Timer timer, FILE* fptr) {
    double fps = get_fps(timer);
    fprintf(fptr, "\n======================================\nMarker : %s\nCompute Time : %ld ms\nFPS : %f\n======================================\n", 
        timer.marker.c_str(), 
        timer.elapsed_ms,
        fps
    );
}

