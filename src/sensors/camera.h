// camera.h - Camera sensor: takes a picture of the road in front of the car.
// Uses your own photos from a folder if there are any, otherwise it draws the scene itself
// (see camera_render.h). Pictures are big (~675 KB), so this sensor produces most of the data.
// Priority in the project: MEDIUM.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "../world.h"

namespace sensors {

constexpr double kCameraFps = 10.0;   // pictures per second

struct CameraFrame {
    uint32_t frameNo = 0;
    uint64_t timeUs = 0;
    world::Sighting sees;            // the obstacle in front of the car (same area the LIDAR checks)
    std::string imageName;           // e.g. "frame_00012.bmp" or the photo's file name
    std::string extension;           // ".bmp", ".jpg" or ".png"
    std::vector<uint8_t> image;      // the complete image file
    bool hasBox = false;             // drawn pictures only: where `sees` is in the picture
    int box[4] = {0, 0, 0, 0};       // x0, y0, x1, y1 in pixels

    size_t sizeBytes() const { return 96 + image.size(); }
};

class Camera {
public:
    explicit Camera(const std::string& photoDir);

    CameraFrame capture(const world::World& world, uint64_t timeUs, uint32_t frameNo) const;
    bool usingPhotos() const { return !photos_.empty(); }
    size_t photoCount() const { return photos_.size(); }

private:
    struct Photo {
        std::string name, extension;
        std::vector<uint8_t> data;
    };
    const Photo& choosePhoto(const world::Sighting& sees, uint32_t frameNo) const;

    std::vector<Photo> photos_;
};

}  // namespace sensors
