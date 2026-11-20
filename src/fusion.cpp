// fusion.cpp
#include "fusion.h"

const char* toString(Decision d) {
    switch (d) {
        case Decision::Brake: return "BRAKE";
        case Decision::Slow: return "SLOW";
        default: return "CRUISE";
    }
}

Decision decide(const sensors::LidarScan& lidar) {
    if (lidar.nearest < 10.0f) return Decision::Brake;
    if (lidar.nearest < 25.0f) return Decision::Slow;
    return Decision::Cruise;
}
