// options.cpp
#include "options.h"

#include <iostream>
#include <stdexcept>

namespace {

void printUsage() {
    std::cerr << "Usage: sensor_sim.exe [options]\n"
                 "  --duration SEC     simulated seconds, 0 = until stopped (default 30)\n"
                 "  --speed X          1 = real time, 0 = as fast as possible (default 1)\n"
                 "  --dashboard        send live data to tools/dashboard.py (JSON lines on stdout)\n"
                 "  --no-csv           do not write the CSV files\n"
                 "  --save-images N    save every Nth camera picture, 0 = none (default 10)\n"
                 "  --photos DIR       folder with your own camera photos (default assets/camera)\n"
                 "  --out DIR          output folder (default sim_output)\n"
                 "  --seed N           obstacle layout (default 42)\n";
}

}  // namespace

bool parseOptions(int argc, char** argv, Options& o) {
    try {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            auto value = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error("missing value after " + arg);
                return argv[++i];
            };
            if (arg == "--duration") o.durationSec = std::stod(value());
            else if (arg == "--speed") o.speed = std::stod(value());
            else if (arg == "--dashboard") o.dashboard = true;
            else if (arg == "--no-csv") o.saveCsv = false;
            else if (arg == "--save-images") o.saveImageEvery = std::stoi(value());
            else if (arg == "--photos") o.photoDir = value();
            else if (arg == "--out") o.outDir = value();
            else if (arg == "--seed") o.seed = (unsigned)std::stoul(value());
            else if (arg == "--help" || arg == "-h") { printUsage(); return false; }
            else throw std::runtime_error("unknown option " + arg);
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n\n";
        printUsage();
        return false;
    }
    return true;
}
