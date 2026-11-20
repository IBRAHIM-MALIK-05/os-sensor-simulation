// gps.h - GPS sensor: reports where the car is as latitude/longitude, with a little noise
// (a real GPS is only accurate to a metre or two).
// Priority in the project: LOW (background logger).
#pragma once

#include <cstddef>
#include <cstdint>
#include <random>

#include "../world.h"

namespace sensors {

constexpr double kGpsHz = 5.0;                             // fixes per second
constexpr double kStartLat = 33.6844, kStartLon = 73.0479;   // where the road starts (x = 0)

struct GpsFix {
    uint32_t fixNo = 0;
    uint64_t timeUs = 0;
    double lat = 0, lon = 0;   // degrees
    double speedMps = 0;
    double headingDeg = 0;     // compass heading (90 = east)
    double accuracyM = 1.5;

    static constexpr size_t sizeBytes() { return 64; }
};

// Converts a position in the world (metres) to latitude/longitude.
void toLatLon(double x, double y, double& lat, double& lon);

class Gps {
public:
    explicit Gps(unsigned seed) : rng_(seed), noise_(0.0, 1.5) {}
    GpsFix read(const world::World& world, uint64_t timeUs, uint32_t fixNo);

private:
    std::mt19937 rng_;
    std::normal_distribution<double> noise_;   // metres
};

}  // namespace sensors
