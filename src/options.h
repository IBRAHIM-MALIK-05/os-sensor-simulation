// options.h - command-line options of sensor_sim.exe
#pragma once

#include <string>

struct Options {
    double durationSec = 30;                 // simulated seconds, 0 = run until stopped
    double speed = 1.0;                      // 1 = real time, 2 = twice as fast, 0 = as fast as possible
    bool dashboard = false;                  // send JSON lines to tools/dashboard.py on stdout
    bool saveCsv = true;                     // write the CSV files
    int saveImageEvery = 10;                 // save every Nth camera picture (0 = none)
    std::string photoDir = "assets/camera";  // your own camera photos (optional)
    std::string outDir = "sim_output";
    unsigned seed = 42;                      // changes where the obstacles are
};

// Returns false (after printing the problem and the usage) if the arguments are wrong.
bool parseOptions(int argc, char** argv, Options& options);
