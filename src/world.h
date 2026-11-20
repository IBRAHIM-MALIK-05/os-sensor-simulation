// world.h - the simulated world that all three sensors look at.
//
// A car drives along a 1 km road that loops (after 1000 m it starts again at 0 m).
// Obstacles (cars, trucks, pedestrians, cyclists, cones and signs) stand on and beside the road.
// Everything is calculated from the time alone, so the world never changes after it is created
// and any thread can read it at any moment without locks.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace world {

struct Obstacle {
    int id;
    std::string kind;   // "car", "truck", "pedestrian", "cyclist", "cone", "sign"
    double x, y;        // position in metres: x = along the road, y = across it (+ = left)
    double radius;      // size in metres, as the LIDAR sees it
};

// Where our car is and where it is going.
struct Pose {
    double x, y;      // metres
    double heading;   // radians, 0 = straight along the road
    double speed;     // metres per second
};

// An obstacle seen in front of the car.
struct Sighting {
    bool found = false;
    int obstacleId = -1;
    std::string kind;
    double distance = 0;     // metres
    double bearingDeg = 0;   // angle from straight ahead, + = left
};

class World {
public:
    static constexpr double kRoadLength = 1000.0;   // metres, then the road loops
    static constexpr double kCarSpeed = 12.0;       // metres per second (about 43 km/h)
    static constexpr double kRoadHalfWidth = 7.6;   // metres from the centre to the edge line

    explicit World(unsigned seed);

    Pose carPose(uint64_t timeUs) const;
    double laneY(double x) const;   // where the car drives across the road at position x

    // Distance from (x, y) along a ray at `angle` to the first obstacle, or maxRange if nothing.
    double castRay(double x, double y, double angle, double maxRange) const;

    // The closest obstacle within +-halfAngleDeg in front of the car.
    Sighting nearestAhead(const Pose& car, double halfAngleDeg, double maxRange) const;

    // Offset from (x, y) to an obstacle, using whichever copy of the looping road is closest.
    void offsetTo(const Obstacle& o, double x, double y, double& dx, double& dy) const;

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }

private:
    std::vector<Obstacle> obstacles_;
};

}  // namespace world
