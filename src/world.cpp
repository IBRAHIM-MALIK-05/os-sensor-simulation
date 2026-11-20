// world.cpp
#include "world.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace world {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct ObstacleType {
    const char* kind;
    double radius;   // metres
    double share;    // how common this type is (all shares add up to 1)
};

const ObstacleType kTypes[] = {
    {"car", 1.0, 0.30},     {"truck", 1.35, 0.12}, {"pedestrian", 0.35, 0.16},
    {"cyclist", 0.5, 0.14}, {"cone", 0.3, 0.16},   {"sign", 0.25, 0.12},
};

const ObstacleType& pickType(double random01) {
    for (const auto& type : kTypes) {
        if (random01 < type.share) return type;
        random01 -= type.share;
    }
    return kTypes[0];
}

}  // namespace

World::World(unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> gap(25.0, 45.0), across(-7.0, 7.0), random01(0.0, 1.0);

    int id = 0;
    for (double x = 40.0; x < kRoadLength - 30.0; x += gap(rng)) {
        const ObstacleType& type = pickType(random01(rng));
        Obstacle o{id++, type.kind, x, 0.0, type.radius};
        // keep at least 2 m between the obstacle and the car's driving line, so the car never hits it
        do {
            o.y = across(rng);
        } while (std::fabs(o.y - laneY(o.x)) < o.radius + 2.0);
        obstacles_.push_back(o);
    }
}

// The car weaves gently between lanes: 3 lane changes per lap, the same path every lap.
double World::laneY(double x) const { return 2.5 * std::sin(2 * kPi * 3 * x / kRoadLength); }

Pose World::carPose(uint64_t timeUs) const {
    double seconds = (double)timeUs / 1e6;
    Pose p;
    p.speed = kCarSpeed;
    p.x = std::fmod(kCarSpeed * seconds, kRoadLength);
    p.y = laneY(p.x);
    double k = 2 * kPi * 3 / kRoadLength;
    p.heading = std::atan(2.5 * k * std::cos(k * p.x));   // slope of the driving line
    return p;
}

void World::offsetTo(const Obstacle& o, double x, double y, double& dx, double& dy) const {
    dx = o.x - x;
    if (dx > kRoadLength / 2) dx -= kRoadLength;
    if (dx < -kRoadLength / 2) dx += kRoadLength;
    dy = o.y - y;
}

double World::castRay(double x, double y, double angle, double maxRange) const {
    double ux = std::cos(angle), uy = std::sin(angle);
    double closest = maxRange;
    for (const auto& o : obstacles_) {
        double dx, dy;
        offsetTo(o, x, y, dx, dy);
        if (std::fabs(dx) > maxRange + o.radius) continue;   // too far away to matter
        // ray-circle intersection
        double along = dx * ux + dy * uy;
        if (along < 0) continue;   // behind the ray
        double sideways2 = dx * dx + dy * dy - along * along;
        if (sideways2 > o.radius * o.radius) continue;   // the ray misses it
        double hit = std::max(0.0, along - std::sqrt(o.radius * o.radius - sideways2));
        closest = std::min(closest, hit);
    }
    return closest;
}

Sighting World::nearestAhead(const Pose& car, double halfAngleDeg, double maxRange) const {
    Sighting best;
    for (const auto& o : obstacles_) {
        double dx, dy;
        offsetTo(o, car.x, car.y, dx, dy);
        if (dx <= 0) continue;   // behind us
        double distance = std::max(0.0, std::sqrt(dx * dx + dy * dy) - o.radius);
        if (distance > maxRange) continue;
        double bearing = (std::atan2(dy, dx) - car.heading) * 180.0 / kPi;
        if (std::fabs(bearing) > halfAngleDeg) continue;
        if (!best.found || distance < best.distance) best = {true, o.id, o.kind, distance, bearing};
    }
    return best;
}

}  // namespace world
