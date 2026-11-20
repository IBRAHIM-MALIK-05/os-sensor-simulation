// disk_recorder.h - saves every sensor reading to files in the output folder:
//   lidar.csv     one row per LIDAR scan (all 720 distances)
//   camera.csv    one row per camera picture (what it sees)
//   gps.csv       one row per GPS fix
//   combined.csv  one row per LIDAR scan with the latest reading of all three sensors + decision
//   camera/       every Nth camera picture as an image file
#pragma once

#include <fstream>
#include <string>

#include "../fusion.h"

class DiskRecorder {
public:
    DiskRecorder(const std::string& outDir, bool saveCsv, int saveImageEvery);

    void saveLidar(const sensors::LidarScan& scan);
    void saveCamera(const sensors::CameraFrame& frame);
    void saveGps(const sensors::GpsFix& fix);
    void saveCombined(const FusedState& state);

private:
    std::string outDir_;
    bool saveCsv_;
    int saveImageEvery_;
    std::ofstream lidarCsv_, cameraCsv_, gpsCsv_, combinedCsv_;
    uint32_t combinedRows_ = 0;
};
