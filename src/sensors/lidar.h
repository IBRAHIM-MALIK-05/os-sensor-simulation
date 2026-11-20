// lidar.h - LIDAR sensor: measures the distance to obstacles all around the car with lasers.
// Priority in the project: HIGH (safety critical, hard real-time).
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "../world.h"

namespace sensors {

constexpr double kLidarHz = 10.0;          // scans per second
constexpr int kLidarRays = 720;            // one distance every 0.5 degrees, full 360 degrees
constexpr double kLidarRange = 60.0;       // metres; no obstacle closer than this = "nothing there"
constexpr double kFrontHalfAngle = 30.0;   // "in front of the car" means within +-30 degrees

struct LidarScan {
    uint32_t scanNo = 0;
    uint64_t timeUs = 0;
    world::Pose car{};
    std::vector<float> ranges;      // kLidarRays distances, ranges[i] is at angle i * 0.5 deg (0 = ahead, + = left)
    float nearest = kLidarRange;    // closest distance in front of the car
    float nearestBearingDeg = 0;    // and its angle (+ = left)

    bool seesObstacle() const { return nearest < kLidarRange - 0.01f; }
    size_t sizeBytes() const { return 64 + ranges.size() * sizeof(float); }   // ~2.9 KB
};

LidarScan scanLidar(const world::World& world, uint64_t timeUs, uint32_t scanNo);

}  // namespace sensors
