// disk_recorder.cpp
#include "disk_recorder.h"

#include <cstdio>
#include <filesystem>

namespace {

std::string num(double value, int decimals) {
    char text[40];
    std::snprintf(text, sizeof text, "%.*f", decimals, value);
    return text;
}

}  // namespace

DiskRecorder::DiskRecorder(const std::string& outDir, bool saveCsv, int saveImageEvery)
    : outDir_(outDir), saveCsv_(saveCsv), saveImageEvery_(saveImageEvery) {
    std::filesystem::create_directories(outDir_ + "/camera");
    if (!saveCsv_) return;

    lidarCsv_.open(outDir_ + "/lidar.csv");
    cameraCsv_.open(outDir_ + "/camera.csv");
    gpsCsv_.open(outDir_ + "/gps.csv");
    combinedCsv_.open(outDir_ + "/combined.csv");

    lidarCsv_ << "time_s,scan,car_x_m,car_y_m,nearest_front_m,nearest_bearing_deg";
    for (int i = 0; i < sensors::kLidarRays; i++) lidarCsv_ << ",r" << i;
    lidarCsv_ << "\n";
    cameraCsv_ << "time_s,frame,image,bytes,sees,sees_id,sees_distance_m,sees_bearing_deg,saved_as\n";
    gpsCsv_ << "time_s,fix,lat,lon,speed_mps,heading_deg,accuracy_m\n";
    combinedCsv_ << "time_s,lidar_scan,lidar_nearest_m,lidar_bearing_deg,camera_frame,camera_sees,camera_distance_m,"
                    "gps_fix,lat,lon,decision\n";
}

void DiskRecorder::saveLidar(const sensors::LidarScan& scan) {
    if (!saveCsv_) return;
    lidarCsv_ << num(scan.timeUs / 1e6, 2) << "," << scan.scanNo << "," << num(scan.car.x, 2) << ","
              << num(scan.car.y, 2) << "," << num(scan.nearest, 2) << "," << num(scan.nearestBearingDeg, 1);
    for (float r : scan.ranges) lidarCsv_ << "," << num(r, 2);
    lidarCsv_ << "\n";
}

void DiskRecorder::saveCamera(const sensors::CameraFrame& frame) {
    std::string savedAs;
    if (saveImageEvery_ > 0 && frame.frameNo % (uint32_t)saveImageEvery_ == 0) {
        char name[48];
        std::snprintf(name, sizeof name, "/camera/frame_%05u%s", frame.frameNo, frame.extension.c_str());
        savedAs = outDir_ + name;
        std::ofstream(savedAs, std::ios::binary)
            .write(reinterpret_cast<const char*>(frame.image.data()), (std::streamsize)frame.image.size());
    }
    if (!saveCsv_) return;
    cameraCsv_ << num(frame.timeUs / 1e6, 2) << "," << frame.frameNo << "," << frame.imageName << ","
               << frame.image.size() << "," << (frame.sees.found ? frame.sees.kind : "none") << ","
               << frame.sees.obstacleId << "," << num(frame.sees.distance, 2) << "," << num(frame.sees.bearingDeg, 1)
               << "," << savedAs << "\n";
}

void DiskRecorder::saveGps(const sensors::GpsFix& fix) {
    if (!saveCsv_) return;
    gpsCsv_ << num(fix.timeUs / 1e6, 2) << "," << fix.fixNo << "," << num(fix.lat, 7) << "," << num(fix.lon, 7) << ","
            << num(fix.speedMps, 1) << "," << num(fix.headingDeg, 1) << "," << num(fix.accuracyM, 1) << "\n";
}

void DiskRecorder::saveCombined(const FusedState& s) {
    if (!saveCsv_) return;
    combinedCsv_ << num(s.timeSec, 2) << "," << s.lidar.scanNo << "," << num(s.lidar.nearest, 2) << ","
                 << num(s.lidar.nearestBearingDeg, 1) << "," << s.camera.frameNo << ","
                 << (s.camera.sees.found ? s.camera.sees.kind : "none") << "," << num(s.camera.sees.distance, 2) << ","
                 << s.gps.fixNo << "," << num(s.gps.lat, 7) << "," << num(s.gps.lon, 7) << "," << toString(s.decision)
                 << "\n";
    // write to disk every second, so the files are up to date even if the program is stopped
    if (++combinedRows_ % 10 == 0) {
        lidarCsv_.flush();
        cameraCsv_.flush();
        gpsCsv_.flush();
        combinedCsv_.flush();
    }
}
