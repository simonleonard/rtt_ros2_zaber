#pragma once

#include <sstream>
#include <string>

struct Command {
    Command(const std::string& line) {
        std::istringstream ss(line);
        double start_time_s;
        ss >> joint >> start_time_s >> target >> velocity;
        start_time = start_time_s * 1000000000;
    }

    std::string joint;
    long start_time;
    double target;
    double velocity;
};
