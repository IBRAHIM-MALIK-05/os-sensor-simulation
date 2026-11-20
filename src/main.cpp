// main.cpp - sensor simulation of a self-driving car.
//
// Three sensors look at the same world:
//   LIDAR   10 scans per second    distances all around the car   ~2.9 KB each
//   Camera  10 pictures per second  the road in front               ~675 KB each
//   GPS      5 fixes per second     latitude / longitude            64 B each
//
// This file is the timeline: it moves simulated time forward, asks each sensor for a reading when
// it is due, keeps the newest reading of every sensor (FusedState), combines them into a driving
// decision, and hands everything to the outputs (files on disk and/or the live dashboard).
//
// Today everything runs in this one loop. In the OS project each sensor becomes a producer thread
// that puts its readings into a shared buffer, and the combining step becomes the consumer thread.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>

#include "fusion.h"
#include "options.h"
#include "output/dashboard_stream.h"
#include "output/disk_recorder.h"
#include "sensors/camera.h"
#include "sensors/gps.h"
#include "sensors/lidar.h"
#include "world.h"

namespace {

// Waits so that simulated time keeps pace with the real clock (speed 1 = real time).
class Pacer {
public:
    explicit Pacer(double speed) : speed_(speed), start_(std::chrono::steady_clock::now()) {}
    void waitUntil(uint64_t simTimeUs) const {
        if (speed_ > 0)
            std::this_thread::sleep_until(start_ + std::chrono::microseconds((long long)(simTimeUs / speed_)));
    }

private:
    double speed_;
    std::chrono::steady_clock::time_point start_;
};

void printStatus(const FusedState& s) {
    char lidar[48], camera[64];
    if (s.lidar.seesObstacle()) std::snprintf(lidar, sizeof lidar, "%.1f m ahead", s.lidar.nearest);
    else std::snprintf(lidar, sizeof lidar, "clear");
    if (s.camera.sees.found)
        std::snprintf(camera, sizeof camera, "%s at %.1f m", s.camera.sees.kind.c_str(), s.camera.sees.distance);
    else std::snprintf(camera, sizeof camera, "clear road");
    std::printf("%6.1f s | LIDAR: %-14s | Camera: %-22s | GPS: %.5f, %.5f | %s\n", s.timeSec, lidar, camera, s.gps.lat,
                s.gps.lon, toString(s.decision));
    std::fflush(stdout);
}

void printSummary(const DataCounters& c, double seconds, const sensors::Camera& camera, const Options& o) {
    std::printf("\nDone. %.1f simulated seconds:\n", seconds);
    std::printf("  LIDAR   %5u scans    %8.1f KB   (%.1f KB/s)\n", c.lidarScans, c.lidarBytes / 1024.0,
                c.lidarBytes / 1024.0 / seconds);
    std::printf("  Camera  %5u pictures %8.1f MB   (%.2f MB/s, %s)\n", c.cameraFrames, c.cameraBytes / 1048576.0,
                c.cameraBytes / 1048576.0 / seconds, camera.usingPhotos() ? "your photos" : "drawn pictures");
    std::printf("  GPS     %5u fixes    %8.1f KB\n", c.gpsFixes, c.gpsBytes / 1024.0);
    double bytesPerSec = (c.lidarBytes + c.cameraBytes + c.gpsBytes) / seconds;
    std::printf("  A 10 MB buffer would hold %.1f s of this data.\n", 10.0 * 1048576.0 / bytesPerSec);
    if (o.saveCsv) std::printf("Data saved in %s\\ (lidar.csv, camera.csv, gps.csv, combined.csv)\n", o.outDir.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) return 1;

    // ---- the world and the three sensors ----
    world::World world(options.seed);
    sensors::Camera camera(options.photoDir);
    sensors::Gps gps(options.seed + 1);

    // ---- where the data goes ----
    DiskRecorder disk(options.outDir, options.saveCsv, options.saveImageEvery);
    std::unique_ptr<DashboardStream> dashboard;
    if (options.dashboard) {
        dashboard = std::make_unique<DashboardStream>(std::cout, options.outDir);
        dashboard->sendWorld(world);
    }

    // ---- timeline ----
    const uint64_t lidarEvery = (uint64_t)(1e6 / sensors::kLidarHz);    // microseconds
    const uint64_t cameraEvery = (uint64_t)(1e6 / sensors::kCameraFps);
    const uint64_t gpsEvery = (uint64_t)(1e6 / sensors::kGpsHz);
    const uint64_t endTime = options.durationSec > 0 ? (uint64_t)(options.durationSec * 1e6)
                                                     : std::numeric_limits<uint64_t>::max();
    uint64_t nextLidar = 0, nextCamera = 0, nextGps = 0, nextStatus = 0;

    FusedState latest;      // newest reading of every sensor
    DataCounters counters;
    Pacer pacer(options.speed);

    while (true) {
        uint64_t now = std::min({nextLidar, nextCamera, nextGps});   // the next thing that happens
        if (now >= endTime) break;
        pacer.waitUntil(now);

        if (now == nextGps) {
            latest.gps = gps.read(world, now, counters.gpsFixes++);
            counters.gpsBytes += sensors::GpsFix::sizeBytes();
            disk.saveGps(latest.gps);
            nextGps += gpsEvery;
        }

        if (now == nextCamera) {
            latest.camera = camera.capture(world, now, counters.cameraFrames++);
            counters.cameraBytes += latest.camera.sizeBytes();
            disk.saveCamera(latest.camera);
            if (dashboard) dashboard->saveLiveImage(latest.camera);
            nextCamera += cameraEvery;
        }

        if (now == nextLidar) {
            latest.lidar = sensors::scanLidar(world, now, counters.lidarScans++);
            counters.lidarBytes += latest.lidar.sizeBytes();
            disk.saveLidar(latest.lidar);
            nextLidar += lidarEvery;

            // combine the newest readings and decide (the future AI processor)
            latest.timeSec = now / 1e6;
            latest.decision = decide(latest.lidar);
            disk.saveCombined(latest);

            if (dashboard) {
                dashboard->sendState(latest, counters, camera.usingPhotos());
                if (!dashboard->connected()) return 0;   // dashboard was closed
            } else if (now >= nextStatus) {
                printStatus(latest);   // one line per simulated second
                nextStatus += 1000000;
            }
        }
    }

    if (dashboard) dashboard->sendEnd();
    else printSummary(counters, options.durationSec, camera, options);
    return 0;
}
