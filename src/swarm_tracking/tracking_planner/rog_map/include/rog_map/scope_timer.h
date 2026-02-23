#ifndef _ROGMAP_TIMER_H_
#define _ROGMAP_TIMER_H_
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>

namespace benchmark_utils {
    class TimeConsuming {

    public:
        TimeConsuming();

        TimeConsuming(std::string msg, int repeat_time);

        TimeConsuming(std::string msg, bool print_log = true);

        ~TimeConsuming();

        void set_enbale(bool enable);

        void start();

        double stop();

    private:
        std::chrono::high_resolution_clock::time_point tc_start, tc_end;
        std::string msg_;
        int repeat_time_{1};
        bool has_shown = false;
        bool enable_{true};
        bool print_{true};
    };
}

#endif
