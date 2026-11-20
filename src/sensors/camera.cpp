// camera.cpp
#include "camera.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "camera_render.h"
#include "lidar.h"   // kFrontHalfAngle, kLidarRange: the camera checks the same area as the LIDAR

namespace fs = std::filesystem;

namespace sensors {

namespace {

constexpr uint64_t kMaxPhotoBytes = 8ull << 20;

std::string lowercase(std::string s) {
    for (auto& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}

}  // namespace

Camera::Camera(const std::string& photoDir) {
    std::error_code ec;
    if (!fs::is_directory(photoDir, ec)) return;

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(photoDir, ec)) {
        std::string ext = lowercase(entry.path().extension().string());
        if (entry.is_regular_file() && (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp"))
            files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());

    for (const auto& file : files) {
        std::ifstream in(file, std::ios::binary);
        Photo photo{file.filename().string(), file.extension().string(),
                    {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()}};
        if (!photo.data.empty() && photo.data.size() <= kMaxPhotoBytes) photos_.push_back(std::move(photo));
    }
}

// Picks a photo that matches what is in front of the car: a photo whose file name contains the
// obstacle type (e.g. "truck_1.jpg") if there is one, otherwise one chosen by the obstacle number.
const Camera::Photo& Camera::choosePhoto(const world::Sighting& sees, uint32_t frameNo) const {
    if (sees.found) {
        for (const auto& photo : photos_)
            if (lowercase(photo.name).find(sees.kind) != std::string::npos) return photo;
        return photos_[(size_t)sees.obstacleId % photos_.size()];
    }
    return photos_[frameNo % photos_.size()];
}

CameraFrame Camera::capture(const world::World& world, uint64_t timeUs, uint32_t frameNo) const {
    CameraFrame frame;
    frame.frameNo = frameNo;
    frame.timeUs = timeUs;
    world::Pose car = world.carPose(timeUs);
    frame.sees = world.nearestAhead(car, kFrontHalfAngle, kLidarRange);

    if (usingPhotos()) {
        const Photo& photo = choosePhoto(frame.sees, frameNo);
        frame.imageName = photo.name;
        frame.extension = photo.extension;
        frame.image = photo.data;
    } else {
        char name[32];
        std::snprintf(name, sizeof name, "frame_%05u.bmp", frameNo);
        frame.imageName = name;
        frame.extension = ".bmp";
        RenderedImage picture = renderDashcam(world, car, frame.sees.found ? frame.sees.obstacleId : -1);
        frame.image = std::move(picture.bmp);
        frame.hasBox = picture.hasBox;
        std::copy(picture.box, picture.box + 4, frame.box);
    }
    return frame;
}

}  // namespace sensors
