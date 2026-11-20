// gps.cpp
#include "gps.h"

#include <cmath>

namespace sensors {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerDegree = 111320.0;
}  // namespace

void toLatLon(double x, double y, double& lat, double& lon) {
    lat = kStartLat + y / kMetresPerDegree;
    lon = kStartLon + x / (kMetresPerDegree * std::cos(kStartLat * kPi / 180.0));
}

GpsFix Gps::read(const world::World& world, uint64_t timeUs, uint32_t fixNo) {
    world::Pose car = world.carPose(timeUs);
    GpsFix fix;
    fix.fixNo = fixNo;
    fix.timeUs = timeUs;
    toLatLon(car.x + noise_(rng_), car.y + noise_(rng_), fix.lat, fix.lon);
    fix.speedMps = car.speed;
    fix.headingDeg = 90.0 - car.heading * 180.0 / kPi;   // the road points east
    return fix;
}

}  // namespace sensors
