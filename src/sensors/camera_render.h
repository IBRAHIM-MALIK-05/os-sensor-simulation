// camera_render.h - draws the dash-cam picture when no photos are used:
// sky, hills, the road with lane markings, and every obstacle in view as a recognisable object.
#pragma once

#include <cstdint>
#include <vector>

#include "../world.h"

namespace sensors {

constexpr int kImageWidth = 640, kImageHeight = 360;

struct RenderedImage {
    std::vector<uint8_t> bmp;    // complete .bmp file (24-bit)
    bool hasBox = false;         // was the focus obstacle in the picture?
    int box[4] = {0, 0, 0, 0};   // its bounding box: x0, y0, x1, y1 in pixels
};

// focusObstacleId: the obstacle to put a box around (-1 = none)
RenderedImage renderDashcam(const world::World& world, const world::Pose& car, int focusObstacleId);

}  // namespace sensors
