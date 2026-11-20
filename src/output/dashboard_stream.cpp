// dashboard_stream.cpp
#include "dashboard_stream.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "../sensors/camera_render.h"

namespace fs = std::filesystem;

namespace {

std::string num(double value, int decimals) {
    if (!std::isfinite(value)) return "null";
    char text[40];
    std::snprintf(text, sizeof text, "%.*f", decimals, value);
    return text;
}

std::string str(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += ((unsigned char)c < 0x20) ? ' ' : c;
    }
    return out + "\"";
}

}  // namespace

DashboardStream::DashboardStream(std::ostream& out, const std::string& outDir) : out_(out), liveDir_(outDir + "/live") {
    fs::create_directories(liveDir_);
}

void DashboardStream::sendWorld(const world::World& world) {
    double lat0, lon0;
    sensors::toLatLon(0, 0, lat0, lon0);
    std::string j = "{\"type\":\"world\",\"length\":" + num(world::World::kRoadLength, 1) +
                    ",\"lidarRange\":" + num(sensors::kLidarRange, 1) + ",\"road\":{\"lat0\":" + num(lat0, 7) +
                    ",\"lon0\":" + num(lon0, 7) + "},\"obstacles\":[";
    for (size_t i = 0; i < world.obstacles().size(); i++) {
        const auto& o = world.obstacles()[i];
        j += (i ? "," : "");
        j += "{\"id\":" + std::to_string(o.id) + ",\"kind\":" + str(o.kind) + ",\"x\":" + num(o.x, 2) +
             ",\"y\":" + num(o.y, 2) + ",\"r\":" + num(o.radius, 2) + "}";
    }
    out_ << j << "]}\n" << std::flush;
}

void DashboardStream::saveLiveImage(const sensors::CameraFrame& frame) {
    std::string path = liveDir_ + "/live_" + std::to_string(frame.frameNo % 2) + frame.extension;
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(reinterpret_cast<const char*>(frame.image.data()), (std::streamsize)frame.image.size());
    file.close();
    if (file) liveImagePath_ = fs::absolute(path).string();
}

void DashboardStream::sendState(const FusedState& s, const DataCounters& c, bool usingPhotos) {
    const auto& lidar = s.lidar;
    const auto& cam = s.camera;
    const auto& gps = s.gps;
    double elapsed = s.timeSec + 1.0 / sensors::kLidarHz;   // seconds of data so far

    std::string j;
    j.reserve(8000);
    j += "{\"type\":\"state\",\"t\":" + num(s.timeSec, 2);
    j += ",\"car\":{\"x\":" + num(lidar.car.x, 2) + ",\"y\":" + num(lidar.car.y, 2) +
         ",\"heading\":" + num(lidar.car.heading * 180 / 3.14159265358979, 2) + ",\"speed\":" + num(lidar.car.speed, 1) + "}";

    j += ",\"lidar\":{\"scan\":" + std::to_string(lidar.scanNo) + ",\"nearest\":" + num(lidar.nearest, 2) +
         ",\"bearing\":" + num(lidar.nearestBearingDeg, 1) + ",\"bytes\":" + std::to_string(lidar.sizeBytes()) +
         ",\"ranges\":[";
    for (size_t i = 0; i < lidar.ranges.size(); i++) j += (i ? "," : "") + num(lidar.ranges[i], 2);
    j += "]}";

    j += ",\"camera\":{\"frame\":" + std::to_string(cam.frameNo) + ",\"image\":" + str(cam.imageName) +
         ",\"bytes\":" + std::to_string(cam.image.size()) + ",\"file\":" + str(liveImagePath_) +
         ",\"source\":" + str(usingPhotos ? "photos" : "synthetic") + ",\"w\":" + std::to_string(sensors::kImageWidth) +
         ",\"h\":" + std::to_string(sensors::kImageHeight) + ",\"sees\":";
    j += cam.sees.found ? "{\"kind\":" + str(cam.sees.kind) + ",\"id\":" + std::to_string(cam.sees.obstacleId) +
                              ",\"dist\":" + num(cam.sees.distance, 2) + ",\"bearing\":" + num(cam.sees.bearingDeg, 1) + "}"
                        : std::string("null");
    j += ",\"box\":";
    j += cam.hasBox ? "[" + std::to_string(cam.box[0]) + "," + std::to_string(cam.box[1]) + "," +
                          std::to_string(cam.box[2]) + "," + std::to_string(cam.box[3]) + "]"
                    : std::string("null");
    j += "}";

    j += ",\"gps\":{\"fix\":" + std::to_string(gps.fixNo) + ",\"lat\":" + num(gps.lat, 7) + ",\"lon\":" + num(gps.lon, 7) +
         ",\"speed\":" + num(gps.speedMps, 1) + ",\"heading\":" + num(gps.headingDeg, 1) + ",\"acc\":" + num(gps.accuracyM, 1) + "}";

    j += ",\"decision\":" + str(toString(s.decision));
    j += ",\"counts\":{\"lidar\":" + std::to_string(c.lidarScans) + ",\"camera\":" + std::to_string(c.cameraFrames) +
         ",\"gps\":" + std::to_string(c.gpsFixes) + "}";
    j += ",\"rates\":{\"lidar\":" + num(c.lidarBytes / elapsed, 0) + ",\"camera\":" + num(c.cameraBytes / elapsed, 0) +
         ",\"gps\":" + num(c.gpsBytes / elapsed, 1) + "}}";

    out_ << j << "\n" << std::flush;
}

void DashboardStream::sendEnd() { out_ << "{\"type\":\"end\"}\n" << std::flush; }
