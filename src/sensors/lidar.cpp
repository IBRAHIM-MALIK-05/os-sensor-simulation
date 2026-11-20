// lidar.cpp
#include "lidar.h"

#include <cmath>

namespace sensors {

LidarScan scanLidar(const world::World& world, uint64_t timeUs, uint32_t scanNo) {
    const double kPi = 3.14159265358979323846;
    LidarScan scan;
    scan.scanNo = scanNo;
    scan.timeUs = timeUs;
    scan.car = world.carPose(timeUs);
    scan.ranges.resize(kLidarRays);

    for (int i = 0; i < kLidarRays; i++) {
        double angle = i * (2 * kPi / kLidarRays);   // relative to the car, counter-clockwise
        float distance = (float)world.castRay(scan.car.x, scan.car.y, scan.car.heading + angle, kLidarRange);
        scan.ranges[i] = distance;

        double angleDeg = angle * 180.0 / kPi;
        if (angleDeg > 180) angleDeg -= 360;   // -180 .. 180
        if (std::fabs(angleDeg) <= kFrontHalfAngle && distance < scan.nearest) {
            scan.nearest = distance;
            scan.nearestBearingDeg = (float)angleDeg;
        }
    }
    return scan;
}

}  // namespace sensors
